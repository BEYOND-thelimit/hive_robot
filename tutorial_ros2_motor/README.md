# tutorial_ros2_motor
Modified from CrashLab motor control lesson for use

# Install pigpio
```markdown
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

# How to set pigpio
```markdown
// 모터 노드를 실행하기 전에 1회 실행해 줄 것
sudo pigpiod 

//최초 pigpio 설치시 제대로 설치되었는지 확인
sudo ./x_pigpiod_if2

//pigpiod 비정상일시, 멈추고 다시 시작할 때 사용
sudo killall pigpiod
```

# Use Motor Code
```markdown
sudo pigpiod
ros2 run tutorial_ros2_motor motor_node
```
