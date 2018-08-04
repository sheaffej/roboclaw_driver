set -e

WS_TARGET_REPO_PATH=$CATKIN_WORKSPACE/src/$TARGET_REPO_NAME

echo "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="
echo "|   Running ici_after_script.sh   |"
echo "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="
echo

echo "==== whoami ===="
whoami
echo

echo "==== pwd ===="
pwd
echo

echo "==== env ===="
env
echo

echo "==== install pytest & coveralls ===="
pip install -q pytest pytest-cov coveralls
echo

echo "==== prep for catkin_make ===="
cd $CATKIN_WORKSPACE
rm -Rf build devel
unlink $CATKIN_WORKSPACE/src/$TARGET_REPO_NAME
cp -a $TARGET_REPO_PATH $CATKIN_WORKSPACE/src/
ls -l $CATKIN_WORKSPACE/src/
echo

echo "==== WS_TARGET_REPO_PATH ===="
echo $WS_TARGET_REPO_PATH
echo

echo "==== catkin_make ===="
cd $CATKIN_WORKSPACE
catkin_make
source $CATKIN_WORKSPACE/devel/setup.bash
echo

echo "===== rospack find $TARGET_REPO_NAME ===="
rospack find $TARGET_REPO_NAME

echo "==== run_tests ===="
cd $WS_TARGET_REPO_PATH
source $WS_TARGET_REPO_PATH/tests/run_tests.sh
echo

echo "==== coveralls ===="
if [[ -n $TRAVIS ]]; then
    cd $WS_TARGET_REPO_PATH
    coveralls
else
    echo "Not running in Travis-CI...skipping coveralls"
fi
echo

# echo "=============================="
# echo "==== rostest test results ===="
# echo "=============================="
# echo
# ROSLOGS=`find $HOME/.ros/test_results/$TARGET_REPO_NAME -name "rostest-*.xml" -print`
# for LOG in $ROSLOGS; do
#     echo "== $LOG =="
#     cat $LOG
#     echo; echo
# done