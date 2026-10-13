#!/usr/bin/env bash
# シミュレーションと AI ノードをすべて止める
pkill -INT -f "[r]os2 launch ai_patrol"
pkill -f "[l]ib/ai_patrol/"
pkill -f "[i]mage_bridge"
pkill -x MicroXRCEAgent
pkill -f "[b]in/px4"
pkill -f "[g]z sim"
pkill -f "[t]ruth_logger.py"
echo "停止しました"
