#!/bin/bash

exec > /home/user/Desktop/autostart_log.txt 2>&1

echo "--- 자동 시작 스크립트 실행됨: $(date) ---"

echo "1. 현재 폴더 이동 시도: /home/user/Desktop/"
cd /home/user/Desktop/
echo "   > 현재 위치: $(pwd)"

source /home/user/Desktop/venv/bin/activate

python3 /home/user/Desktop/gui_control_gtk.py
echo "--- 스크립트 종료됨 ---"
