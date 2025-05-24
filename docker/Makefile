.PHONY: build run shell up stop start down clean

#build the docker image defined in docker-compose.yml
build:
	docker compose build

#start one-time interactive container, auto-removed when exit
run:
	docker compose run --rm ros2-dev bash

#attach a shell to the running container (must already be ran via `up`)
shell:
	docker compose exec ros2-dev bash

#start the container in detached mode (runs in background) to keep container
#running persistently
up:
	docker compose up -d

#stop the running the container started via up
stop:
	docker compose stop

#restart a container that was stopped
start:
	docker compose start

#stop and remove everything created by up
down:
	docker compose down

#remove all stopped containers from your system
clean:
	docker container prune -f
	docker image prune -f
	docker network prune -f
