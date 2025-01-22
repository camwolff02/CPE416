# Add the name of the image for this command to work!
# CONTAINER_NAME := arm64v8/ros:humble
CONTAINER_NAME := ambulantelab/cpe416:lab4-arm

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
	-v /Users/camwolff/Development/CPE416/:/CPE416/:Z \
	${CONTAINER_NAME} \

arch:
	@echo "Make will pull the following image: ${CONTAINER_NAME}"
