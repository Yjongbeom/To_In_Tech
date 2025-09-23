export DISPLAY=:0
exec > /home/user/Desktop/autostart_log.txt 2>&1

echo "--- 자동 시작 스크립트 실행됨: $(date) ---"

echo "1. 현재 폴더 이동 시도: /home/user/Desktop/"
cd /home/user/Desktop/
echo "   > 현재 위치: $(pwd)"

echo "2. 가상 환경 활성화 시도..."
source venv/bin/activate
echo "   > 파이썬 위치: $(which python)"

echo "3. 파이썬 프로그램 실행 시도..."
python gui_control_gtk.py

echo "--- 스크립트 종료됨 ---"