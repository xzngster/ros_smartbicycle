#!/usr/bin/env python3

import rospy
from playsound import playsound
from std_msgs.msg import Int32MultiArray

SOUNDFILE = "/home/ubuntu/catkin_ws/src/mysound/soundfile/sound_one/" # 사운드파일 경로
FILENAME = ("left/left", "leftback/leftback", "back/back", "rightback/rightback", "right/right") # 사운드파일 이름
warning = [0,0,0,0,0] # 출력할 위험 신호
chk = True # 소리 출력 가능한 상태

def soundplay():
        chk = False # 소리출력 시작
        min_dist = 10 # 최소거리 저장
        idx = -1 # 방향 저장
        for i in range(5): # 방향 돌면서
                if warning[i] > 0 and min_dist > warning[i]: # 가장 가까운 거리 찾고
                        min_dist = warning[i] # 최소거리
                        idx = i # 방향
        if idx > -1: # 위험신호가 있으면
                playsound(SOUNDFILE+FILENAME[idx]+str(min_dist)+"m.wav") # 사운드 출력
                print(FILENAME[idx] + " sound is activated, dist :", min_dist) # 터미널에 사운드 출력 표시
                warning[idx] = 0 # 위험신호 초기화
        chk = True # 소리출력 끝

def direction_callback(data): # 위험정보 수신
        print("receved...") # 터미널에 수신 완료 표시
        for i in range(5): # 가장 가까운 거리로 warning을 초기화
                if data.data[i] > 0 and (warning[i] == 0 or warning[i] > data.data[i]): # 최소거리라면
                        warning[i] = data.data[i] # 최소거리 저장

def main():
        rospy.init_node('mysound', anonymous=True) # 노드 초기화
        sub = rospy.Subscriber('direction', Int32MultiArray, direction_callback) # 위험정보 수신
        rate = rospy.Rate(10) # 10Hz 프로그램
        while not rospy.is_shutdown(): # 메인 루프
                if chk: # 소리 출력 가능한 상태라면
                        soundplay() # 소리 출력
                rate.sleep() # 10Hz 맞추기 위해 대기
        rospy.spin() # 프로그램이 종료되는 것을 막음

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
