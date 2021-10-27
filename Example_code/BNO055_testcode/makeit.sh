# gcc -O2 -lm Lab6_student.c -o Lab6_student `pkg-config --cflags --libs opencv`
# gcc -O2 -lm Camera.c -o Camera `pkg-config --cflags --libs opencv`
# gcc -O2 -lm i2c.c Lab6_student_RC.c -o Lab6_student_RC `pkg-config --cflags --libs opencv`
gcc -O2 -lm getbno055.c i2c_bno055.c -o BNO055_Read
gcc -O2 -lm getbno.c i2c_bno055.c -o BNO055_test_prompt
