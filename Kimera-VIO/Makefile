all: help

help:
	@echo ""
	@echo "------------------- Help Menu ------------------------"
	@echo ""
	@echo "     1. make build             - build all images"
	@echo "     2. make pull              - pull all images"
	@echo "     3. make clean             - remove all images"
	@echo ""
	@echo "------------------------------------------------------"
	@echo ""

build:
	@docker build --tag kopanev/kimera:version1 -f ./Dockerfile ..

pull:
	@docker pull kopanev/kimera:version1

clean:
	@docker rmi -f kopanev/kimera:version1
