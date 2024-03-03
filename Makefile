build:
	sudo docker build -t morphology .

nvidia:
	sudo docker run -it --gpus all --mount type=bind,source="$(shell pwd)",destination=/app morphology
