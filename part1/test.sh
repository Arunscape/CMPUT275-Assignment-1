python3 server.py < test00-input.txt > mysol.txt
diff mysol.txt test00-output.txt

python3 server.py < test01-input.txt > mysol.txt
diff mysol.txt test01-output.txt

rm mysol.txt
