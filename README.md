```
apt-get update
apt-get -y dist-upgrade
apt-get -y install gcc make bc screen ncurses-dev
```

```
cd /usr/src
git clone -b rpi-3.18.y --depth 1 git://github.com/raspberrypi/linux.git
ln -s /usr/src/linux /lib/modules/3.6.11+/build
```

```
cd /lib/modules/3.6.11+/build
make mrproper
gzip -dc /proc/config.gz > .config
make modules_prepare
```

```
wget https://raw.githubusercontent.com/raspberrypi/firmware/614599c9d1f6b75d93e02c0252eaa8e286d1c98f/extra/Module.symvers
```

```
depmod -a
```

```
cd /usr/src
git clone -b rpi-3.18.y --depth 1 git://github.com/jpg-consulting/kmod-huawei-voice.git
```

```
cd /usr/src/kmod-huawei-voice
make
```

Install module

```
make install
```
