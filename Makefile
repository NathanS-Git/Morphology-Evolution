build:
	sudo docker build -t morphology .

nvidia:
	sudo docker run -e WANDB_RUN_GROUP=$(shell date) -it --gpus all --mount type=bind,source="$(shell pwd)",destination=/app morphology
