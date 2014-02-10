.PHONY: collect-jars
collect-jars: all
	mkdir -p jars
	rm -f jars/*jar
	cp $(shell grep ros.runtime.jarfileset ros.properties|cut -d '=' -f 2|tr ',' ' ') jars
	cp $(shell grep ros.artifact.built ros.properties|cut -d '=' -f 2) jars

include $(shell rospack find rosjava_bootstrap)/rosjava.mk
