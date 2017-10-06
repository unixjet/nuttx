openocd -f $1 \
-c init -c targets \
-c "halt" \
-c "flash write_image erase nuttx" \
-c "verify_image nuttx" \
-c "reset run" \
-c shutdown
