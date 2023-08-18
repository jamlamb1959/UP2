ifdef BRANCH_NAME
BNAME=${BRANCH_NAME}
else
BNAME=$(shell basename ${PWD})
endif

BROKER="pharmdata.ddns.net"
DEST=pharmdata.ddns.net
PLATFORM=esp32dev
PROJ=UP2

all: .stateflow push

clean:
	pio run --target clean

.mkdir:
	@echo "ssh ${DEST} \"mkdir -p /var/www/html/firmware/$(BNAME)/${PLATFORM}/${PROJ}\""
	@ssh ${DEST} "mkdir -p /var/www/html/firmware/$(BNAME)/${PLATFORM}/${PROJ}"
	touch .mkdir

push: .mkdir
	@echo "PROJ: ${PROJ}"
	@echo "PLATFORM: ${PLATFORM}"
	@echo "DEST: ${DEST}"
	PATH=${PATH}:${HOME}/.local/bin ; pio run
	scp .pio/build/${PLATFORM}/firmware.bin ${DEST}:/var/www/html/firmware/$(BNAME)/${PLATFORM}/${PROJ}/firmware.bin
	mosquitto_pub -h ${BROKER} -t "/MGMT_UP2" -m "reboot"

upload:
	pio run --target upload

monitor:
	pio device monitor

data/SIM7000A.stateflow: StateFlow/SIM7000A.stateflow
	cp -f StateFlow/SIM7000A.stateflow data/SIM7000A.stateflow

.stateflow: data/SIM7000A.stateflow
	scp StateFlow/SIM7000A.stateflow ${DEST}:/var/www/html/StateFlow/SIM7000A.stateflow
	mosquitto_pub -h ${BROKER} -t "/MGMT_UP2" -m "reboot"

mu: .stateflow upload monitor




