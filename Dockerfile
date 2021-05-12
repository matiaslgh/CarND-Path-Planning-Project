FROM gcc:11.1.0

EXPOSE 4567

WORKDIR /usr/src/app

RUN apt-get update && \
  apt-get install -y cmake libgtest-dev libboost-test-dev libuv1-dev libssl-dev libz-dev && \
  rm -rf /var/lib/apt/lists/* && \
  git clone https://github.com/uWebSockets/uWebSockets && \
  cd uWebSockets && \
  git checkout e94b6e1 && \
  mkdir build && \
  cd build && \
  cmake .. && \
  make && \
  make install && \
  cd .. && \
  cd .. && \
  ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so && \
  rm -r uWebSockets

COPY . .

RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make

WORKDIR /usr/src/app/build

CMD ["./path_planning"]