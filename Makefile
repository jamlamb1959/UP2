ifdef BRANCH_NAME
BNAME=${BRANCH_NAME}
else
BNAME=$(shell basename ${PWD})
endif

DEST=repo.sheepshed.tk
PLATFORM=esp32dev
PROJ=UP2

all: mkdir push

clean:
	pio run --target clean

mkdir:
	@echo "ssh ${DEST} \"mkdir -p /var/www/html/firmware/$(BNAME)/${PLATFORM}/${PROJ}\""
	@ssh ${DEST} "mkdir -p /var/www/html/firmware/$(BNAME)/${PLATFORM}/${PROJ}"

push:
	@echo "PROJ: ${PROJ}"
	@echo "PLATFORM: ${PLATFORM}"
	@echo "DEST: ${DEST}"
	env | sort
	export PATH=${PATH}:${HOME}/.local/bin
	env | sort
	pio run
	scp .pio/build/${PLATFORM}/firmware.bin ${DEST}:/var/www/html/firmware/$(BNAME)/${PLATFORM}/${PROJ}/firmware.bin

upload:
	pio run --target upload

monitor:
	pio device monitor

mu: upload monitor




