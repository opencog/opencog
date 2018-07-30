FROM ubuntu:16.04

RUN apt-get update && apt-get install -y \
    python3-pip \
	git \
    build-essential \
    cmake \
    cxxtest \
    rlwrap \
    libiberty-dev \
    libicu-dev \
    libbz2-dev \
    cython \
    python3-dev \
    python3-zmq \
    python3-simplejson \
    python3-nose \
    python-nose \
    libboost-date-time-dev \
    libboost-filesystem-dev \
    libboost-math-dev \
    libboost-program-options-dev \
    libboost-regex-dev \
    libboost-serialization-dev \
    libboost-thread-dev \
    libboost-system-dev \
    libboost-random-dev \
    libzmq3-dev \
    libtbb-dev \
    binutils-dev \
    unixodbc-dev \
    libpq-dev \
    uuid-dev \
    libprotoc-dev \
    protobuf-compiler \
    libsdl-gfx1.2-dev \
    libssl-dev \
    tcl-dev \
    tcsh \
    libfreetype6-dev \
    libatlas-base-dev \
    gfortran \
    gearman \
    libgearman-dev \
    ccache \
    libgsasl7 \
    libldap2-dev \
    krb5-multidev \
    wordnet \
    wordnet-dev \
    wordnet-sense-index \
    libatomic-ops-dev\
    libgmp-dev \
    libffi-dev \
    libreadline-dev \
    doxygen \
    automake

# Guile 2.2+ is not in 16.04 repos

# Guile dependencies
RUN apt-get install -y \
    libunistring-dev \
    libgc-dev \
    wget

# Fetch, build, and install guile
RUN cd /tmp && wget -q https://ftp.gnu.org/gnu/guile/guile-2.2.4.tar.gz \
    && tar xfz guile-2.2.4.tar.gz \
    && cd guile-2.2.4 \
    && ./configure \
    && make \
    && make install

# Additions for link-grammar
RUN apt-get install -y locales autoconf-archive swig flex graphviz-dev
RUN (echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
     echo "ru_RU.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "he_IL.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "de_DE.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "lt_LT.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "fa_IR.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "ar_AE.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "kk_KZ.UTF-8 UTF-8" >> /etc/locale.gen && \
     echo "tr_TR.UTF-8 UTF-8" >> /etc/locale.gen)
RUN locale-gen


