import os
import cv2
import numpy as np
import time
from time import sleep
from picamera2 import Picamera2
from ultralytics import YOLO
import RPi.GPIO as GPIO
import smbus

##############################
# GPIO 핀 설정 - 선풍기 및 제어 관련
IN1 = 23  # 모터 드라이버 IN1 핀
IN2 = 24  # 모터 드라이버 IN2 핀

# GPIO 핀 설정 - 부저 및 LED 관련
BUZZER_PIN = 6  # 부저 핀 (BCM 모드)
LED_PINS = [17, 27, 22]  # LED 핀 (BCM 모드)

# 서보모터 핀 설정
SERVO1_PIN = 12  # 물리적 핀 32
SERVO2_PIN = 13  # 물리적 핀 33

##############################
# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_PINS, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(SERVO1_PIN, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

# 서보모터 초기화
servo1 = GPIO.PWM(SERVO1_PIN, 50)
servo2 = GPIO.PWM(SERVO2_PIN, 50)
servo1.start(7.5)
servo2.start(7.5)
sleep(2)
servo1.ChangeDutyCycle(0)
servo2.ChangeDutyCycle(0)

# PWM 설정 (부저 및 모터)
pwm = GPIO.PWM(BUZZER_PIN, 1000)
pwm.start(0)  # 부저 초기화 (소리 꺼짐 상태)
pwmA = GPIO.PWM(IN1, 800)  # 모터 PWM 설정
pwmA.start(0)

##############################
# YOLOv8 모델 로드
yolo_model_path = '/home/pi/capstone/model/gesture_recognition_model4.pt'
model = YOLO(yolo_model_path)

# Picamera2 설정
camera = Picamera2()
camera.configure(camera.create_preview_configuration({"size": (640, 640)}))  # 프레임 해상도 설정
camera.start()  # 카메라 시작

# 동작 상태 관리 변수 초기화
action_states = {"scissors": False, "rock": False, "paper": False}  # 각 동작의 ON/OFF 상태
last_action_time = time.time()  # 마지막 동작 시간
DELAY_TIME = 2  # 동일 동작 무시 대기 시간 (초)

##############################
# 선풍기 동작 함수
def motor_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)  # 속도 조절

def motor_stop():
    pwmA.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

##############################
# 부저 제어 함수
def control_buzzer(duration):
    pwm.start(50)  # 부저 소리 켜기
    time.sleep(duration)
    pwm.stop()  # 부저 끄기

# 전원 제어 함수 (LED와 부저 제어)
def control_led_and_buzzer(state):
    if state == 1:  # 전원 켬
        print("LED On")
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.HIGH)  # LED 켜기
        control_buzzer(0.5)  # 부저 켜기
    elif state == -1:  # 전원 끔
        print("LED Off")
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.LOW)  # LED 끄기
        control_buzzer(0.5)  # 부저 끄기

# 서보모터 동작 함수
def control_servo(action):
    if action == "open":
        print("Servo Open")
        control_buzzer(0.5)  # 부저 켜기
        servo1.ChangeDutyCycle(2.5)
        servo2.ChangeDutyCycle(14.5)
        sleep(2)
        servo1.ChangeDutyCycle(0)
        servo2.ChangeDutyCycle(0)
    elif action == "close":
        print("Servo Close")
        control_buzzer(0.5)  # 부저 켜기
        servo1.ChangeDutyCycle(7.5)
        servo2.ChangeDutyCycle(7.5)
        sleep(2)
        servo1.ChangeDutyCycle(0)
        servo2.ChangeDutyCycle(0)

##############################
# LCD 관련 함수
I2C_ADDR = 0x27  # LCD 모듈의 I2C 주소
LCD_WIDTH = 16   # LCD의 문자 수 (16x2 LCD의 경우)
LCD_CHR = 1  # 문자 모드
LCD_CMD = 0  # 명령 모드
LCD_LINE_1 = 0x80  # LCD 첫 번째 줄
LCD_LINE_2 = 0xC0  # LCD 두 번째 줄
LCD_BACKLIGHT = 0x08  # 백라이트 ON
ENABLE = 0b00000100   # Enable 비트
bus = smbus.SMBus(1)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(0.0005)

def lcd_init():
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.0005)

def lcd_string(message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for char in message:
        lcd_byte(ord(char), LCD_CHR)

##############################
# 실시간 인식 루프
try:
    while True:
        start_time = time.time()
        frame = camera.capture_array()
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        results = model(frame, stream=False)

        for result in results:
            boxes = result.boxes.xyxy
            class_ids = result.boxes.cls
            confidences = result.boxes.conf

            for box, conf, cls in zip(boxes, confidences, class_ids):
                if time.time() - last_action_time < DELAY_TIME:
                    continue

                # 박스 정보 가져오기

                x1, y1, x2, y2 = map(int, box.tolist())
                label = model.names[int(cls)]
                confidence = conf.item()

                # 초록색 박스와 인식률 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 초록색 박스
                text = f"{label}: {confidence * 100:.1f}%"  # 동작 이름과 인식률
                cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if label in action_states and confidence >= 0.7:
                    action_states[label] = not action_states[label]  # 상태 반전 (ON <-> OFF)
                    last_action_time = time.time()

                    if label == "scissors":  # 선풍기 제어
                        lcd_init()
                        lcd_string("Scissors detected", LCD_LINE_1)
                        if action_states[label]:
                            control_buzzer(0.5)
                            motor_forward(30)
                        else:
                            motor_stop()
                            control_buzzer(0.5)

                    elif label == "rock":  # LED 제어
                        lcd_init()
                        lcd_string("Rock detected", LCD_LINE_1)
                        if action_states[label]:
                            control_led_and_buzzer(1)
                        else:
                            control_led_and_buzzer(-1)

                    elif label == "paper":  # 서보모터 제어
                        lcd_init()
                        lcd_string("Paper detected", LCD_LINE_1)
                        if action_states[label]:
                            control_servo("open")
                        else:
                            control_servo("close")

        # FPS 계산 및 출력
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 화면에 표시
        cv2.imshow("YOLOv8 Hand Gesture Recognition", frame)

        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    camera.stop()
    cv2.destroyAllWindows()
    for pin in LED_PINS:
        GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()
    print("GPIO pins have been reset.")
