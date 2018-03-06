#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Unit-level tests
echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
pushd $DIR >/dev/null
pytest test_roboclaw_stub_unit.py
popd > /dev/null

# Node-level tests
echo
echo "============================"
echo "     Running Node Tests     "
echo "============================"
echo
rostest roboclaw_driver stub.test
