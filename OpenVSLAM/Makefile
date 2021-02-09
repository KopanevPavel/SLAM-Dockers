all: help

help:
	@echo ""
	@echo "------------------- Help Menu ------------------------"
	@echo ""
	@echo "     1. make build             - build all images"
	@echo "     2. make clean             - remove all images"
	@echo ""
	@echo "------------------------------------------------------"
	@echo ""

build:
	@docker build --tag ros:openvslam -f ./Dockerfile ..

clean:
	@docker rmi -f ros:openvslam
