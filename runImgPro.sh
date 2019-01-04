source devel/setup.bash
echo "run image processing..."
roslaunch yolocv_kcf runImgPro.launch
sleep 1
wait
exit 0
