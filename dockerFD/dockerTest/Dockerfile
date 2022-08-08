FROM ubuntu:20.04

LABEL maintainer="Duck"
LABEL version="1.0"
LABEL org.opencontainers.image.authors="duke7295@gmail.com"

COPY . /tmp

WORKDIR /home

ADD test.tar.gz /
ADD test.tar.gz .

ENV PORT=8080

RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get install -y build-essential
RUN apt-get install -y vim

CMD ["echo", "hello world"]

VOLUME ["/home/test_volume"]
# CMD ["ping", "google.com"]

RUN while true; do sleep 1000; done