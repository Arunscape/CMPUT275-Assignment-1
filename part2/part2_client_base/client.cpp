#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>
#include "consts_and_types.h"
#include "map_drawing.h"
#include <stdlib.h>

#include <stdlib.h>//contains the strtol function

// the variables to be shared across the project, they are declared here!
shared_vars shared;

Adafruit_ILI9341 tft = Adafruit_ILI9341(clientpins::tft_cs, clientpins::tft_dc);

void setup() {
  // initialize Arduino
  init();

  // initialize zoom pins
  pinMode(clientpins::zoom_in_pin, INPUT_PULLUP);
  pinMode(clientpins::zoom_out_pin, INPUT_PULLUP);

  // initialize joystick pins and calibrate centre reading
  pinMode(clientpins::joy_button_pin, INPUT_PULLUP);
  // x and y are reverse because of how our joystick is oriented
  shared.joy_centre = xy_pos(analogRead(clientpins::joy_y_pin), analogRead(clientpins::joy_x_pin));

  // initialize serial port
  Serial.begin(9600);
  Serial.flush(); // get rid of any leftover bits

  // initially no path is stored
  shared.num_waypoints = 0;

  // initialize display
  shared.tft = &tft;
  shared.tft->begin();
  shared.tft->setRotation(3);
  shared.tft->fillScreen(ILI9341_BLUE); // so we know the map redraws properly

  // initialize SD card
  if (!SD.begin(clientpins::sd_cs)) {
      Serial.println("Initialization has failed. Things to check:");
      Serial.println("* Is a card inserted properly?");
      Serial.println("* Is your wiring correct?");
      Serial.println("* Is the chipSelect pin the one for your shield or module?");

      while (1) {} // nothing to do here, fix the card issue and retry
  }

  // initialize the shared variables, from map_drawing.h
  // doesn't actually draw anything, just initializes values
  initialize_display_values();

  // initial draw of the map, from map_drawing.h
  draw_map();
  draw_cursor();

  // initial status message
  status_message("FROM?");
}

void process_input() {
  // read the zoom in and out buttons
  shared.zoom_in_pushed = (digitalRead(clientpins::zoom_in_pin) == LOW);
  shared.zoom_out_pushed = (digitalRead(clientpins::zoom_out_pin) == LOW);

  // read the joystick button
  shared.joy_button_pushed = (digitalRead(clientpins::joy_button_pin) == LOW);

  // joystick speed, higher is faster
  const int16_t step = 64;

  // get the joystick movement, dividing by step discretizes it
  // currently a far joystick push will move the cursor about 5 pixels
  xy_pos delta(
    // the funny x/y swap is because of our joystick orientation
    (analogRead(clientpins::joy_y_pin)-shared.joy_centre.x)/step,
    (analogRead(clientpins::joy_x_pin)-shared.joy_centre.y)/step
  );
  delta.x = -delta.x; // horizontal axis is reversed in our orientation

  // check if there was enough movement to move the cursor
  if (delta.x != 0 || delta.y != 0) {
    // if we are here, there was noticeable movement

    // the next three functions are in map_drawing.h
    erase_cursor();       // erase the current cursor
    move_cursor(delta);   // move the cursor, and the map view if the edge was nudged
    if (shared.redraw_map == 0) {
      // it looks funny if we redraw the cursor before the map scrolls
      draw_cursor();      // draw the new cursor position
    }
  }
}

int main() {
  setup();

  // very simple finite state machine:
  // which endpoint are we waiting for?
  enum {WAIT_FOR_START, WAIT_FOR_STOP} curr_mode = WAIT_FOR_START;

  // the two points that are clicked
  lon_lat_32 start, end;

  while (true) {
    // clear entries for new state
    shared.zoom_in_pushed = 0;
    shared.zoom_out_pushed = 0;
    shared.joy_button_pushed = 0;
    shared.redraw_map = 0;

    // reads the three buttons and joystick movement
    // updates the cursor view, map display, and sets the
    // shared.redraw_map flag to 1 if we have to redraw the whole map
    // NOTE: this only updates the internal values representing
    // the cursor and map view, the redrawing occurs at the end of this loop
    process_input();

    // if a zoom button was pushed, update the map and cursor view values
    // for that button push (still need to redraw at the end of this loop)
    // function zoom_map() is from map_drawing.h
    if (shared.zoom_in_pushed) {
      zoom_map(1);
      shared.redraw_map = 1;
    }
    else if (shared.zoom_out_pushed) {
      zoom_map(-1);
      shared.redraw_map = 1;
    }

    // if the joystick button was clicked
    if (shared.joy_button_pushed) {

      if (curr_mode == WAIT_FOR_START) {
        // if we were waiting for the start point, record it
        // and indicate we are waiting for the end point
        start = get_cursor_lonlat();
        curr_mode = WAIT_FOR_STOP;
        status_message("TO?");

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
      else {
        // if we were waiting for the end point, record it
        // and then communicate with the server to get the path
        end = get_cursor_lonlat();

        // TODO: communicate with the server to get the waypoints

        enum State {REQUEST, WAYPOINT};
        State client = REQUEST;

        while (true) {

          if (client == REQUEST) {
            Serial.write('R ');
            Serial.write(start.lat);
            Serial.write(' ');
            Serial.write(start.lon);
            Serial.write(' ');
            Serial.write(end.lat);
            Serial.write(' ');
            Serial.write(end.lon);
            Serial.write('\n');
            Serial.flush();

            char buffer[129];
            int used = 0;

            start = millis();

            // first timeout is 10 seconds
            while (millis()-start < 10000) {
              while (Serial.available() == 0 && millis()-start < 10000);

              buffer[used] = Serial.read();
              ++used;

              // stores start index of number
              int start_index=0;

              if (buffer[used-1] == 'N'){
                start_index = used + 1;
              }
              else if (buffer[used-1] == '\n' && start_index != 0) {

                // reads up to the new line
                shared.num_waypoints = strtol(&buff[start_index],NULL,10);
                Serial.write('A');
                Serial.write('\n');
                Serial.flush();
                client = WAYPOINT;
                break;
              }

            }

          else if (client == WAYPOINT) {
            

          }

          else if (client == END) {
            break;
          }

        }



        // now we have stored the path length in
        // shared.num_waypoints and the waypoints themselves in
        // the shared.waypoints[] array, switch back to asking for the
        // start point of a new request

        //The code for reading from seria will be something like:

        // WORKS IN THEORY NEED TO TEST THIS
        //handles drawing the route
        for(i=0,i<shared.num_waypoints,i++){
          uint8_t	byteRead = Serial.read();
          char lineRead[];
          count=0;
          while (byteRead != "\n"){ //lines are separated by newline \n character
            lineRead[count] = byteRead;
            byteRead = Serial.read();
            count++;
          }

          //thanks to Jason Cannon for the idea to use strtol()
          char* pointer; //helps to separate the string by space
          shared.waypoints[i].lat= strtol(&lineRead, &pointer, 10);
          shared.waypoints[i].lon= strtol(&pointer, NULL, 10);

        //TODO make a variable that tells me if a valid route was found plz

        routefound=true;
        //~Arun

        curr_mode = WAIT_FOR_START;

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
    }

    if (shared.redraw_map) {
      // redraw the status message
      if (curr_mode == WAIT_FOR_START) {
        status_message("FROM?");
      }
      else {
        status_message("TO?");
      }

      // redraw the map and cursor
      draw_map();
      draw_cursor();

      // TODO: draw the route if there is one
      if(route){//if a valid route was found
        // In particular, you are mostly concerned with
        //  - shared.num_waypoints: the number of waypoints on the path
        //  - shared.waypoints[]: the lat/lon pairs of these waypoints
        //  - max_waypoints (a global const, not in the shared_vars struct):
        //    the maximum number of waypoints that can be stored in the
        //    shared.waypoints[] array
        for(i=0,i<shared.num_waypoints,i++){
          Serial.println(shared.waypoints[i]) //wtf does this look like?

          // in case I fuck something up
          // x0=shared.waypoints[i].lat
          // y0-shared.waypoints[i].lon
          //
          // x1=shared.waypoints[i+1].lat
          // y1=shared.waypoints[i+1].lat

          x0=longitude_to_x(map_number,shared.waypoints[i].lat)
          y0=lattitude_to_y(map_number,shared.waypoints[i].lon)

          x1=longitude_to_x(map_number,shared.waypoints[i+1].lat)
          y1=lattitude_to_y(map_number,shared.waypoints[i+1].lat)
          tft.drawLine(x0, y0, x1, y1, 0x001F);//0x001F is BLUE
        }
      }
    }
  }

  Serial.flush();
  return 0;
}
