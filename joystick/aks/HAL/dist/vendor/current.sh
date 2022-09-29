#cat current.sh 
PACKAGE=vendor.aks.gamepad@1.0
CURRENT_PATH="$(dirname $(readlink -f "$0"))"
CURRENT_FILE="./current.txt"
if [ ! -f ${CURRENT_FILE} ]
then
    touch ${CURRENT_FILE}
fi

HASH_VALUE=`hidl-gen -Lhash -rvendor.aks.gamepad:vendor/qcom/opensource/ecosw/gamepad_hal $PACKAGE`
echo $HASH_VALUE >> ${CURRENT_FILE}

echo -e "HASH for current API: \033[41;33m$HASH_VALUE\033[0m"
