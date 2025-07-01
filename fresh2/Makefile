CONTAINER_NAME=ros-noetic_unitree
GPU:=$(shell command -v nvidia-smi >/dev/null 2>&1 && echo true || echo false)
build:
	docker build -t $(CONTAINER_NAME) .

run:
	xhost +local:docker
	-docker rm -f $(CONTAINER_NAME) 2>/dev/null || true
	docker run -it \
		--user robot \
		--hostname $(CONTAINER_NAME) \
		$(if $(filter true, $(GPU)),--gpus all,) \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix \
		--mount type=bind,src=$(CURDIR)/ws,dst=/home/robot/ws \
		--privileged \
		--name $(CONTAINER_NAME) \
		$(CONTAINER_NAME)
stop:
	-docker stop $(CONTAINER_NAME) || true

shell:
	docker exec -it $(CONTAINER_NAME) zsh
clean:
	rm -rf $(CURDIR)/ws/build/
	rm -rf $(CURDIR)/ws/devel/
	rm -rf $(CURDIR)/ws/.catkin_tools/
	rm -rf $(CURDIR)/ws/logs/
	-docker rmi -f $(CONTAINER_NAME) 2>/dev/null || true
rebuild:
	$(MAKE) clean
	docker build --no-cache -t $(CONTAINER_NAME) .
	$(MAKE) run
