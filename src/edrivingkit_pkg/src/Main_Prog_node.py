#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from edrivingkit_pkg.msg import KitCmdMsg, KitFeedMsg
import sys
import termios
import tty
import select
import os
import threading
import time
import curses 

class MainProgNode(Node):
    def __init__(self):
        super().__init__('main_prog_node')
        self.publisher_ = self.create_publisher(KitCmdMsg, 'edrivingkit_cmd', 10)
        self.subscription_ = self.create_subscription(
            KitFeedMsg,
            'edrivingkit_feedback',
            self.feedback_callback,
            10
        )
        self.current_steer_mode = 0  # 0: Manual, 1: Auto
        self.current_steer_angle = 0.0
        self.current_edriving_mode = 0  # 0: Inactive, 1: Active
        self.current_edriving_speed = 0  # RPM
        self.current_estop = 0
        self.nSAS_Zero_set = 0
        self.latest_feedback = None
        self.get_logger().info('Started Main Control Program for eDriving Kit')
        self.settings = termios.tcgetattr(sys.stdin)

    def feedback_callback(self, msg):
        self.latest_feedback = msg

    def display_menu_curses(self, stdscr):
        stdscr.clear()
        stdscr.addstr(1, 2, "eDriving Kit Application".center(56, "="))
        stdscr.addstr(2, 2, "[Current Status]")
        stdscr.addstr(3, 4, f"eSteer angle: {self.current_steer_angle:6.2f}°")
        stdscr.addstr(4, 4, f"eDriving Mode: {'Active' if self.current_estop else 'Inactive'}")
        stdscr.addstr(5, 4, f"eStop Mode: {'Activated' if self.current_estop else 'Deactivated'}")

        if self.latest_feedback:
            stdscr.addstr(6, 2, "[Feedback Information]")
            stdscr.addstr(7, 4, f"Steering Mode: {self.latest_feedback.fd_esteering_mode}")
            stdscr.addstr(8, 4, f"Steering Angle: {self.latest_feedback.fd_esteeringle_angle:.2f}°")
            stdscr.addstr(9, 4, f"eStop Mode: {self.latest_feedback.fd_estop_braking_mode}")
            stdscr.addstr(10, 4, f"eStop Duty: {self.latest_feedback.fd_estop_duty:.1f}")
            
        stdscr.addstr(6, 2, "-"*56)
        stdscr.addstr(7, 2, "[Menu Selection]")
        stdscr.addstr(8, 4, "1. Operating eSteering")
        stdscr.addstr(9, 4, "2. Operating eStop")
        stdscr.addstr(10, 4, "3. Operating eDriving ")
        stdscr.addstr(11, 4, "4. Quit Menu")
        stdscr.addstr(12, 2, "-"*56)
        stdscr.addstr(13, 4, "Please select (1-4): ")
        stdscr.refresh()
        curses.echo()
        choice = stdscr.getstr(13, 4+21, 2).decode().strip()
        curses.noecho()
        return choice

    def control_esteering_menu_curses(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(2, 2, "Which one would you select index in menu?")
            stdscr.addstr(4, 4, "1. Enable eSteering? ")
            stdscr.addstr(5, 4, "2. See the information of total value")
            stdscr.addstr(6, 4, "3. Would like to zero-set angle sensor?")
            stdscr.addstr(7, 4, "4. Go back to Main menu?")
            stdscr.addstr(9, 4, "Input (1-4): ")
            stdscr.refresh()
            curses.echo()
            choice = stdscr.getstr(9, 17, 2).decode().strip()
            curses.noecho()
            if choice == '1':
                self.enable_esteering_mode_menu_curses(stdscr)
            elif choice == '2':
                self.show_total_info_curses(stdscr)
            elif choice == '3':
                self.zero_set_angle_sensor_curses(stdscr)
            elif choice == '4':
                break
            else:
                stdscr.addstr(10, 4, "Please select (1-3): ")
                stdscr.refresh()
                time.sleep(1.5)

    def enable_esteering_mode_menu_curses(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(2, 2, "Select eSteering Mode")
            stdscr.addstr(4, 4, "1. Manual Mode")
            stdscr.addstr(5, 4, "2. Auto Mode")
            stdscr.addstr(6, 4, "Input (1-2): ")
            stdscr.refresh()
            curses.echo()
            choice = stdscr.getstr(6, 17, 2).decode().strip()
            curses.noecho()
            if choice == '1':
                self.current_steer_mode = 0  # Manual mode
                self.current_steer_angle = 0.0
                self.send_command()
                stdscr.addstr(10, 4, "Manual mode has been set.")
                stdscr.refresh()
                time.sleep(1.5)
                self.control_esteering_menu_curses(stdscr)
                break
            elif choice == '2':
                self.current_steer_mode = 1  # Auto mode
                self.current_steer_angle = 0.0
                self.send_command()
                self.control_esteering_angle_menu_curses(stdscr)
                break

    def control_esteering_angle_menu_curses(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(2, 2, "Which one would you select index in menu?")
            stdscr.addstr(4, 4, "1. Go to angle of eSteering(degree):")
            stdscr.addstr(5, 4, "2. See the information of total value")
            stdscr.addstr(6, 4, "3. Go back to previous Menu")
            stdscr.addstr(8, 4, "Input (1-3): ")
            stdscr.refresh()
            curses.echo()
            choice = stdscr.getstr(8, 17, 2).decode().strip()
            curses.noecho()
            if choice == '1':
                self.set_esteering_angle_curses(stdscr)
            elif choice == '2':
                self.show_total_info_curses(stdscr)
            elif choice == '3':
                break
            else:
                stdscr.addstr(10, 4, "Please, select the number (1-3).")
                stdscr.refresh()
                time.sleep(1.5)

    def set_esteering_angle_curses(self, stdscr):
        stdscr.clear()
        stdscr.addstr(2, 2, "Input Steering angle degrees")
        stdscr.addstr(3, 4, "e.g.) -10.0° ~ +10.0°")
        stdscr.addstr(4, 4, "Or press 'q' to return to the previous menu.")
        stdscr.addstr(6, 4, "Input: ")
        stdscr.refresh()
        curses.echo()
        user_input = stdscr.getstr(6, 11, 8).decode().strip()
        curses.noecho()
        if user_input.lower() == 'q':
            return
        try:
            steer_angle = float(user_input)
            if -45.0 <= steer_angle <= 45.0:
                self.current_steer_angle = steer_angle
                self.send_command()
                stdscr.addstr(8, 4, f"Go to {steer_angle}°.")
                stdscr.refresh()
                time.sleep(1.5)
            else:
                stdscr.addstr(8, 4, "Steering angle must be between -10° and 10°.")
                stdscr.refresh()
                time.sleep(1.5)
        except ValueError:
            stdscr.addstr(8, 4, "You must input a valid number.")
            stdscr.refresh()
            time.sleep(1.5)

    def zero_set_angle_sensor_curses(self, stdscr):
        stdscr.clear()
        stdscr.addstr(2, 2, "Are you sure? (y/n)")
        stdscr.refresh()
        curses.echo()
        user_input = stdscr.getstr(2, 21, 8).decode().strip()
        curses.noecho()
        if user_input.lower() == 'n':
            return
        if user_input.lower() == 'y':
            self.nSAS_Zero_set = 0
            self.send_command()
            time.sleep(0.3)

            self.nSAS_Zero_set = 1
            self.send_command()
            time.sleep(0.3)

            self.nSAS_Zero_set = 2
            self.send_command()
            time.sleep(0.3)

            self.nSAS_Zero_set = 0
            self.send_command()
            stdscr.addstr(10, 2, "Zero set completed.")

            time.sleep(0.3)


    def show_total_info_curses(self, stdscr):
        """연속적으로 eSteering 정보를 표시하는 함수 (스레드 활용)"""
        # 키 입력을 위한 설정
        stdscr.nodelay(True)  # 논블로킹 입력 모드
        stdscr.timeout(100)   # 100ms 타임아웃
        
        # 종료 플래그
        exit_flag = threading.Event()
        
        def display_data():
            """데이터를 연속적으로 표시하는 스레드 함수"""
            while not exit_flag.is_set():
                try:
                    stdscr.clear()
                    
                    # 타이틀
                    stdscr.addstr(1, 2, "=== eSteering Information (Real-time) ===")
                    stdscr.addstr(2, 2, "Press any key to exit")
                    stdscr.addstr(3, 2, "-" * 50)
                    
                    # 현재 명령 상태
                    stdscr.addstr(5, 2, "[Current Command Status]")
                    stdscr.addstr(6, 4, f"Steer Mode: {'Auto' if self.current_steer_mode else 'Manual'}")
                    stdscr.addstr(7, 4, f"Target Angle: {self.current_steer_angle:7.2f}°")
                    stdscr.addstr(8, 4, f"eStop Status: {'Activated' if self.current_estop else 'Deactivated'}")
                    
                    # 피드백 데이터
                    if self.latest_feedback:
                        
                        if curses.has_colors():
                            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)  # 정상
                            curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK) # 주의
                            curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)    # 경고
                        
                        stdscr.addstr(10, 2, "[Feedback Information]")
                        stdscr.addstr(11, 4, f"Steering Mode: {self.latest_feedback.fd_esteering_mode} ({'Auto' if self.latest_feedback.fd_esteering_mode else 'Manual'})",curses.color_pair(1))
                        stdscr.addstr(12, 4, f"Actual Angle: {self.latest_feedback.fd_esteeringle_angle:.1f}°",curses.color_pair(1))
                        stdscr.addstr(13, 4, f"eStop Mode: {self.latest_feedback.fd_estop_braking_mode}",curses.color_pair(2))
                        stdscr.addstr(14, 4, f"eStop Duty: {self.latest_feedback.fd_estop_duty:.3f}%",curses.color_pair(3))
                        stdscr.addstr(15, 4, f"eDriving Mode: {self.latest_feedback.fd_edriving_mode}%",curses.color_pair(2))
                        stdscr.addstr(16, 4, f"eDriving Speed: {self.latest_feedback.fd_edriving_speed:.1f} RPM",curses.color_pair(1))
                    else:
                        stdscr.addstr(10, 2, "[Feedback Information]")
                        stdscr.addstr(11, 4, "No feedback data received yet...")
                        stdscr.addstr(12, 4, "Waiting for connection...")
                    
                    # 실시간 타임스탬프
                    current_time = time.strftime("%H:%M:%S", time.localtime())
                    stdscr.addstr(20, 2, f"Last Updated: {current_time}")
                    stdscr.addstr(22, 2, "Press any key to exit")

                    
                    stdscr.refresh()
                    
                except curses.error:
                    # 화면 크기 부족 등의 에러 처리
                    pass
                
                # 100ms 대기
                time.sleep(0.1)
        
        try:
            # 색상 초기화
            if curses.has_colors():
                curses.start_color()
            
            # 데이터 표시 스레드 시작
            display_thread = threading.Thread(target=display_data, daemon=True)
            display_thread.start()
            
            # 키 입력 대기
            while True:
                try:
                    key = stdscr.getch()
                    if key != curses.ERR:  # 키가 눌렸으면
                        break
                except curses.error:
                    pass
                
                # 100ms마다 체크
                time.sleep(0.1)
                
                # ROS가 종료되면 빠져나가기
                if not rclpy.ok():
                    break
            
            # 종료 플래그 설정
            exit_flag.set()
            
            # 스레드 종료 대기 (최대 1초)
            if display_thread.is_alive():
                display_thread.join(timeout=1.0)
            
        except KeyboardInterrupt:
            exit_flag.set()
        
        finally:
            # 설정 복원
            stdscr.nodelay(False)
            stdscr.timeout(-1)

    def control_estop_menu_curses(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(2, 2, "Control eStop")
            stdscr.addstr(4, 4, f"Current Status: {'Activated' if self.current_estop else 'Deactivated'}")
            stdscr.addstr(6, 4, "1. Activate eStop")
            stdscr.addstr(7, 4, "2. Deactivate eStop")
            stdscr.addstr(8, 4, "3. Go back to main menu")
            stdscr.addstr(10, 4, "Input (1/2/3): ")
            stdscr.refresh()
            curses.echo()
            user_input = stdscr.getstr(10, 18, 2).decode().strip()
            curses.noecho()
            if user_input == '3':
                break
            elif user_input == '1':
                self.current_estop = 1
                self.send_command()
                stdscr.addstr(12, 4, "Activated eStop.")
                stdscr.refresh()
                time.sleep(1.5)
            elif user_input == '2':
                self.current_estop = 0
                self.send_command()
                stdscr.addstr(12, 4, "Deactivated eStop.")
                stdscr.refresh()
                time.sleep(1.5)
            else:
                stdscr.addstr(12, 4, "Please, select the number (1-3).")
                stdscr.refresh()
                time.sleep(1.5)

    def control_edriving_menu_curses(self, stdscr):
        while True:
            stdscr.clear()
            stdscr.addstr(2, 2, "Control eDriving")
            stdscr.addstr(4, 4, "1. Activate eDriving")
            stdscr.addstr(5, 4, "2. Deactivate eDriving")
            stdscr.addstr(6, 4, "3. Control speed of Main Motor")
            stdscr.addstr(7, 4, "4. See the information of total value")
            stdscr.addstr(8, 4, "5. Go back to main Menu")
            stdscr.addstr(9, 4, "Input (1/2/3/4/5): ")
            stdscr.refresh()
            curses.echo()
            user_input = stdscr.getstr(9, 25, 2).decode().strip()
            curses.noecho()
            if user_input == '5':
                break
            elif user_input == '1':
                self.current_edriving_mode = 1
                self.send_command()
                stdscr.addstr(10, 4, "Activated eDriving.")
                stdscr.refresh()
                time.sleep(1.5)
            elif user_input == '2':
                self.current_edriving_mode = 0
                self.current_edriving_speed = 0  # Reset speed when deactivating
                self.send_command()
                stdscr.addstr(10, 4, "Deactivated eDriving.")
                stdscr.refresh()
                time.sleep(1.5)
            elif user_input == '3':
                self.set_edriving_speed_curses(stdscr)
            elif user_input == '4':
                self.show_total_info_curses(stdscr)
            else:
                stdscr.addstr(12, 4, "Please, select the number (1-5).")
                stdscr.refresh()
                time.sleep(1.5)

    def set_edriving_speed_curses(self, stdscr):
        stdscr.clear()
        stdscr.addstr(2, 2, "Set eDriving Speed (RPM):")
        stdscr.addstr(3, 4, "Input speed in RPM (0-3000): ")
        stdscr.refresh()
        curses.echo()
        user_input = stdscr.getstr(3, 35, 4).decode().strip()
        curses.noecho()
        try:
            speed = int(user_input)
            if 0 <= speed <= 3000:
                self.current_edriving_speed = speed
                self.send_command()
                stdscr.addstr(5, 4, f"Set eDriving speed to {speed} RPM.")
                stdscr.refresh()
                time.sleep(1.5)
            else:
                stdscr.addstr(5, 4, "Speed must be between 0 and 3000 RPM.")
                stdscr.refresh()
                time.sleep(1.5)
        except ValueError:
            stdscr.addstr(5, 4, "You must input a valid number.")
            stdscr.refresh()
            time.sleep(1.5)


    def send_command(self):
        msg = KitCmdMsg()
        msg.cmd_esteering_mode =    int(self.current_steer_mode)
        msg.cmd_esteering_angle =   float(self.current_steer_angle)
        msg.cmd_edriving_mode =     int(self.current_edriving_mode)
        msg.cmd_edriving_speed =    int(self.current_edriving_speed)  # Convert RPM to integer
        msg.cmd_estop_braking =     int(self.current_estop)
        msg.cmd_esteering_zero_set = self.nSAS_Zero_set
        self.publisher_.publish(msg)
        self.get_logger().info(f'Input Command: Steer_angle={self.current_steer_angle}°, eStop={self.current_estop}')

    def run(self):
        def curses_main(stdscr):
            try:
                while rclpy.ok():
                    choice = self.display_menu_curses(stdscr)
                    if choice == '1':
                        self.control_esteering_menu_curses(stdscr)
                    elif choice == '2':
                        self.control_estop_menu_curses(stdscr)
                    elif choice == '3':
                        self.control_edriving_menu_curses(stdscr)
                    elif choice == '4':
                        stdscr.addstr(30, 2, "Exiting program...")
                        stdscr.refresh()
                        time.sleep(1)
                        break
                    else:
                        stdscr.addstr(32, 2, "Please, select the number (1-3).")
                        stdscr.refresh()
                        time.sleep(1.5)
                        
            except KeyboardInterrupt:
                stdscr.addstr(34, 2, "Keyboard Interrupt detected. Exiting program...")
                stdscr.refresh()
                time.sleep(1)
        curses.wrapper(curses_main)

def main(args=None):
    rclpy.init(args=args)
    node = MainProgNode()
    def spin_thread():
        rclpy.spin(node)
    spin_t = threading.Thread(target=spin_thread, daemon=True)
    spin_t.start()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
