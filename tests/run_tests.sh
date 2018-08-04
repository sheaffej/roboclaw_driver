#!/bin/bash

PKG_DIR=`rospack find roboclaw_driver`

echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
/usr/bin/env pytest -v --cache-clear --cov=roboclaw_driver $PKG_DIR/tests/unit/

echo
echo "============================"
echo "     Running Node Tests     "
echo "============================"
echo
rostest roboclaw_driver stub.test
