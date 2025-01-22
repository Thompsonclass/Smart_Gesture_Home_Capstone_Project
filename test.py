import os
import cv2
import numpy as np
import time
from time import sleep
from picamera2 import Picamera2
from ultralytics import YOLO
import RPi.GPIO as GPIO

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

# 동작 관련 변수 초기화
current_action = "idle"  # 현재 동작 상태
last_action_time = time.time()  # 마지막 동작 시각
DELAY_TIME = 3  # 동작 후 대기 시간 (초)

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

def motor_led_and_buzzer(state):
    if state == 1:  # 전원 켬
        print("Fan Power On")
        control_buzzer(1)  # 부저 켜기
    elif state == -1:  # 전원 끔
        print("Fan Power Off")
        control_buzzer(1)  # 부저 끔

##############################
# 부저 제어 함수
def control_buzzer(duration):
    pwm.start(50)  # 부저 소리 켜기
    time.sleep(duration)
    pwm.stop()  # 부저 끄기

# 전원 제어 함수 (LED와 부저 제어)
def control_led_and_buzzer(state):
    if state == 1:  # 전원 켬
        print("Power On")
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.HIGH)  # LED 켜기
        control_buzzer(1)  # 부저 켜기
    elif state == -1:  # 전원 끔
        print("Power Off")
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.LOW)  # LED 끄기
        control_buzzer(1)  # 부저 끄기
##############################
#LCD
import smbus
import time

# I2C 버스 초기화
I2C_ADDR = 0x27  # LCD 모듈의 I2C 주소
LCD_WIDTH = 16   # LCD의 문자 수 (16x2 LCD의 경우)

# LCD 명령
LCD_CHR = 1  # 문자 모드
LCD_CMD = 0  # 명령 모드

# LCD 줄 위치 정의
LCD_LINE_1 = 0x80  # LCD 첫 번째 줄
LCD_LINE_2 = 0xC0  # LCD 두 번째 줄

# 일부 명령 정의
LCD_BACKLIGHT = 0x08  # 백라이트 ON
ENABLE = 0b00000100   # Enable 비트

# I2C 버스를 초기화합니다.
bus = smbus.SMBus(1)  # Raspberry Pi의 I2C 버스 번호 (1번 버스 사용)

# LCD 명령 전송 함수
def lcd_byte(bits, mode):
    """
    LCD에 데이터(명령 또는 문자)를 전송하는 함수.

    :param bits: 데이터 (8비트)
    :param mode: LCD_CMD (명령) 또는 LCD_CHR (문자)
    """
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT  # 상위 4비트
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT  # 하위 4비트

    # 상위 4비트 전송
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # 하위 4비트 전송
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

# Enable 신호를 토글하여 데이터 전송을 완료합니다.
def lcd_toggle_enable(bits):
    """
    Enable 신호를 짧게 발생시켜 LCD가 데이터를 수신하도록 합니다.
    """
    time.sleep(0.0005)  # 짧은 지연
    bus.write_byte(I2C_ADDR, (bits | ENABLE))  # Enable 비트를 활성화
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))  # Enable 비트를 비활성화
    time.sleep(0.0005)

# LCD 초기화 함수
def lcd_init():
    """
    LCD를 초기화하는 함수. LCD의 기본 설정을 구성합니다.
    """
    lcd_byte(0x33, LCD_CMD)  # 초기화
    lcd_byte(0x32, LCD_CMD)  # 4비트 모드 설정
    lcd_byte(0x06, LCD_CMD)  # 커서 이동 방향
    lcd_byte(0x0C, LCD_CMD)  # 화면 ON, 커서 OFF
    lcd_byte(0x28, LCD_CMD)  # 2줄 모드, 5x8 점 크기
    lcd_byte(0x01, LCD_CMD)  # 화면 지우기
    time.sleep(0.0005)

# LCD에 문자열 출력 함수
def lcd_string(message, line):
    """
    LCD의 특정 줄에 문자열을 출력하는 함수.

    :param message: 출력할 문자열
    :param line: LCD 줄 (LCD_LINE_1 또는 LCD_LINE_2)
    """
    message = message.ljust(LCD_WIDTH, " ")  # 문자열을 16자로 채움
    lcd_byte(line, LCD_CMD)  # 줄 설정

    for char in message:
        lcd_byte(ord(char), LCD_CHR)  # 각 문자를 LCD로 전송

##############################
# 실시간 인식 루프
try:
    while True:
        start_time = time.time()  # FPS 계산용 시간 측정 시작

        # 현재 프레임 캡처
        frame = camera.capture_array()

        # 4채널 BGRA -> 3채널 BGR 변환
        if frame.shape[2] == 4:  # 4채널(BGRA)인 경우
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # YOLO 모델로 예측
        results = model(frame, stream=False)  # 성능 향상을 위해 stream 비활성화

        # 예측 결과를 프레임에 표시
        for result in results:
            boxes = result.boxes.xyxy
            class_ids = result.boxes.cls
            confidences = result.boxes.conf

            # 인식된 객체에 경계 상자 그리기 및 조건 확인
            for box, conf, cls in zip(boxes, confidences, class_ids):
                x1, y1, x2, y2 = map(int, box)
                label = f"{model.names[int(cls)]}: {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 경계 상자
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                # 동작 충돌 방지: DELAY_TIME 동안 다른 동작 금지
                if time.time() - last_action_time < DELAY_TIME:
                    continue

                # 'scissor' 클래스 감지 - 팬 제어
                if model.names[int(cls)] == "scissors" and 0.70 <= conf <= 0.99:
                    print(f"Scissors detected, Confidence: {conf:.2f}")
                    # 프로그램 실행
                    lcd_init()  # LCD 초기화
                    lcd_string("Scissors detected", LCD_LINE_1)  # 첫 번째 줄에 메시지 출력
                    # lcd_string("Confidence: {conf:.2f}", LCD_LINE_2)  # 첫 번째 줄에 메시지 출력                    
                    if current_action != "fan_on":
                        current_action = "fan_on"
                        motor_led_and_buzzer(1)  # 선풍기 켬
                        motor_forward(30)  # 30% 속도로 회전
                    else:
                        current_action = "fan_off"
                        motor_stop()  # 선풍기 끔
                        motor_led_and_buzzer(-1)
                    last_action_time = time.time()

                # 'rock' 클래스 감지 - LED 제어
                elif model.names[int(cls)] == "rock" and 0.70 <= conf <= 0.99:
                    print(f"Rock detected, Confidence: {conf:.2f}")
                    # 프로그램 실행
                    lcd_init()  # LCD 초기화
                    lcd_string("Rock detected", LCD_LINE_1)  # 첫 번째 줄에 메시지 출력
                    # lcd_string("Confidence: {conf:.2f}", LCD_LINE_2)  # 첫 번째 줄에 메시지 출력 
                    if current_action != "led_on":
                        current_action = "led_on"
                        control_led_and_buzzer(1)  # LED 켬
                    else:
                        current_action = "led_off"
                        control_led_and_buzzer(-1)  # LED 끔
                    last_action_time = time.time()

                # 'paper' 클래스 감지 - 서보모터 제어
                elif model.names[int(cls)] == "paper" and 0.70 <= conf <= 0.99:
                    print(f"Paper detected, Confidence: {conf:.2f}")
                    # 프로그램 실행
                    lcd_init()  # LCD 초기화
                    lcd_string("Paper detected", LCD_LINE_1)  # 첫 번째 줄에 메시지 출력
                    # lcd_string("Confidence: {conf:.2f}", LCD_LINE_2)  # 첫 번째 줄에 메시지 출력                    
                    if current_action != "servo_action":
                        current_action = "servo_action"
                        pwm.start(50)  # 부저 소리 켜기
                        time.sleep(1)
                        pwm.stop()
                        servo1.ChangeDutyCycle(2.5)
                        servo2.ChangeDutyCycle(14.5)
                        sleep(2)
                        servo1.ChangeDutyCycle(0)
                        servo2.ChangeDutyCycle(0)
                    else:
                        current_action = "idle"
                        pwm.start(50)  # 부저 소리 켜기
                        time.sleep(1)
                        pwm.stop()
                        servo1.ChangeDutyCycle(7.5)
                        servo2.ChangeDutyCycle(7.5)
                        sleep(2)
                        servo1.ChangeDutyCycle(0)
                        servo2.ChangeDutyCycle(0)
                    last_action_time = time.time()

        # 프레임을 화면에 표시
        cv2.imshow("YOLOv8 Hand Gesture Recognition", frame)

        # FPS 측정
        fps = 1.0 / (time.time() - start_time)
        print(f"FPS: {fps:.2f}")

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 종료 후 자원 해제
    camera.stop()
    cv2.destroyAllWindows()
    for pin in LED_PINS:
        GPIO.output(pin, GPIO.LOW)  # LED 끄기
    GPIO.cleanup()
    print("GPIO pins have been reset.")