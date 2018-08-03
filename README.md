# nao_v5_shri_render

## Requirements

- Install pynaoqi

visit https://developer.softbankrobotics.com. Login and download "Python 2.7 SDK 2.5.10 Linux 64"

        $ tar zxf pynaoqi-python2.7-2.5.7.1-linux64.tar.gz
        $ sudo mkdir -p /usr/local/opt
        $ sudo mv pynaoqi-python2.7-2.5.7.1-linux64 /usr/local/opt/pynaoqi
        $ echo "export PYTHONPATH=/usr/local/opt/pynaoqi/lib/python2.7/site-packages:$PYTHONPATH" >> ~/.bashrc

- Install choregraphe (optional)

visit https://developer.softbankrobotics.com. Login and download "Choregraphe 2.5.10 Linux 64 Setup"

        $ chmod +x choregraphe-suite-2.5.5.5-linux64-setup.run
        $ ./choregraphe-suite-2.5.5.5-linux64-setup.run

- Testing

        $ python
        >>> import naoqi
        <module 'naoqi' from '/usr/local/opt/pynaoqi/lib/python2.7/site-packages/naoqi.pyc'>