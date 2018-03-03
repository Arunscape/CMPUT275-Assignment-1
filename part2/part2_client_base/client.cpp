#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>
#include "consts_and_types.h"
#include "map_drawing.h"

// the variables to be shared across the project, they are declared here!
shared_vars shared;

Adafruit_ILI9341 tft = Adafruit_ILI9341(clientpins::tft_cs, clientpins::tft_dc);

void drawRoute();

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
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
        // if we were waiting for the start point, record it
        // and indicate we are waiting for the end point
        start = get_cursor_lonlat();
        curr_mode = WAIT_FOR_STOP;
        status_message("TO?");

        // wait until the joystick button is no longer pushed

      }
      else {
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
        // if we were waiting for the end point, record it
        // and then communicate with the server to get the path
        end = get_cursor_lonlat();

        // TODO: communicate with the server to get the waypoints
        enum State {REQUEST, WAYPOINT, END};
        State client = REQUEST;
        int waypointCount=0;
        while (true) {

          if (client == REQUEST) {
            char charbuffer[30];
            Serial.write('R');
            Serial.write(' ');
            ltoa(start.lat,charbuffer,10);
            Serial.write(charbuffer);
            Serial.write(' ');
            ltoa(start.lon,charbuffer,10);
            Serial.write(charbuffer);
            Serial.write(' ');
            ltoa(end.lat,charbuffer,10);
            Serial.write(charbuffer);
            Serial.write(' ');
            ltoa(end.lon,charbuffer,10);
            Serial.write(charbuffer);
            Serial.write('\n');
            Serial.flush();

            char buffer[129];
            int used = 0;

            uint32_t startime = millis();
            int start_index=0;

            // first timeout is 10 seconds
            while (millis()-startime < 10000) {
              while (Serial.available() == 0 && millis()-startime < 10000);

              buffer[used] = Serial.read();
              ++used;


              if (buffer[used-1] == 'N'){
                start_index = used + 1;
              }
              else if (buffer[used-1] == '\n' && start_index != 0) {

                // reads up to the new line
                shared.num_waypoints = strtol(&buffer[start_index],NULL,10);
                Serial.write('A');
                Serial.write('\n');
                Serial.flush();
                client = WAYPOINT;
                waypointCount=0;
                break;
              }

            }



          }

          else if (client == WAYPOINT) {
            // now we have stored the path length in
            // shared.num_waypoints and the waypoints themselves in
            // the shared.waypoints[] array, switch back to asking for the
            // start point of a new request

            //The code for reading from seria will be something like:

            // WORKS IN THEORY NEED TO TEST THIS
            //handles drawing the route

            uint32_t startime = millis();
            char lineRead[129];
            int byteInLine=0;
            int start_index=0;
            bool timeout = true;

            while (millis()-startime < 1000 && timeout){ //lines are separated by newline \n character
              while (Serial.available() == 0 && millis()-startime < 10000);
              uint8_t	byteRead = Serial.read();
              lineRead[byteInLine] = byteRead;

              if (byteRead == 'W') {
                start_index = byteInLine + 2;
              }
              else if (byteRead == '\n' && start_index != 0) {
                timeout = false;
                break;
              }

              byteInLine++;
            }

            if (timeout == true) {
              client = REQUEST;
            }
            else {
              Serial.write('A');
              Serial.write('\n');
              Serial.flush();
              //thanks to Jason Cannon for the idea to use strtol()
              char* pointer; //helps to separate the string by space
              shared.waypoints[waypointCount].lat= strtol(&lineRead[start_index], &pointer, 10);
              shared.waypoints[waypointCount].lon= strtol(pointer, NULL, 10);
              waypointCount++;
            }
            tft.setTextSize(1);
            // tft.fillScreen(0);
            tft.setCursor(10*waypointCount,15);
            tft.println(waypointCount);
            if (shared.num_waypoints == waypointCount) {
              client = END;
            }

          }

          else if (client == END) {

            char buffer[129];
            int used = 0;


            uint32_t startime = millis();
            bool finished = false;

            while (millis()-startime < 1000) {
              while (Serial.available() == 0 && millis()-startime < 1000);

              buffer[used] = Serial.read();
              ++used;

              if (buffer[used-2] == 'E' && buffer[used-1] == '\n'){
                finished = true;
                // tft.fillScreen(0);
                break;
              }
            }

            if (finished) {
              shared.redraw_map=1;
              break;
            }
            else {
              client = REQUEST;
            }

          }



        }


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

      //    shared.waypoints[] array
      // tft.fillScreen(0);
      drawRoute();
    }
  }

  Serial.flush();
  return 0;
}

void drawRoute(){
  for(int i=0; i<shared.num_waypoints-1; i++){
    // in case I fuck something up
    // x0=shared.waypoints[i].lat
    // y0-shared.waypoints[i].lon
    //
    // x1=shared.waypoints[i+1].lat
    // y1=shared.waypoints[i+1].lat

    int x0=longitude_to_x(shared.map_number,shared.waypoints[i].lon)-shared.map_coords.x;
    int y0=latitude_to_y(shared.map_number,shared.waypoints[i].lat)-shared.map_coords.y;

    int x1=longitude_to_x(shared.map_number,shared.waypoints[i+1].lon)-shared.map_coords.x;
    int y1=latitude_to_y(shared.map_number,shared.waypoints[i+1].lat)-shared.map_coords.y;
    shared.tft->drawLine(x0, y0, x1, y1, 0x001F);//0x001F is BLUE
  }
}
