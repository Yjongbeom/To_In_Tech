export DISPLAY=:0
exec > /home/jongbeom/Desktop/autostart_log.txt 2>&1

echo "--- 자동 시작 스크립트 실행됨: $(date) ---"

echo "1. 현재 폴더 이동 시도: /home/jongbeom/Desktop/"
cd /home/jongbeom/Desktop/
echo "   > 현재 위치: $(pwd)"

/home/jongbeom/Desktop/.venv/bin/python /home/jongbeom/Desktop/gui_control_gtk.py

echo "--- 스크립트 종료됨 ---"