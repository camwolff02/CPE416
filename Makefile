# Add the name of the image for this command to work!
ifeq ($(shell uname -p), x86_64)
	CONTAINER_NAME := ambulantelab/cpe416:lab4-x86
else
	CONTAINER_NAME := ambulantelab/cpe416:lab4-arm
endif

build: 
	docker build . -t ${IMAGE_NAME}

novnc:
	docker run -d --rm --net=ros \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc -p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --name ${NAME} \
	--net=ros \
	--env="DISPLAY=novnc:0.0" \
	-v ~/Development/CPE416/:/CPE416/:Z \
	${CONTAINER_NAME} \

arch:
	@echo "Make will pull the following image: ${CONTAINER_NAME}"
