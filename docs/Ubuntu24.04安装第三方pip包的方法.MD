# Ubuntu24.04安装第三方pip包的方法.MD

根据在系统环境下执行pip3 install piper_sdk给出的报错提示，现在的版本，使用官方python3.12-venv创建的虚拟环境，然后在虚拟环境中安装piper_sdk是可以的

Based on the error message from executing `pip3 install piper_sdk` in the system environment, in the current version, it is possible to install `piper_sdk` within a virtual environment created using the official Python 3.12-venv.

```bash
sudo apt install python3-pip
```

```bash
sudo apt install python3.12-venv
```

```bash
python3 -m venv ~/venv
```

```bash
source ~/venv/bin/activate
```

```bash
pip install pyAgxArm
```

你也可以强制安装

You can also force installation

通过添加 --break-system-packages 标志来绕过。

Bypass this by adding the `--break-system-packages` flag.

```bash
pip install --user pyAgxArm --break-system-packages
```

无需每次运行 pip install 命令时都键入 --break-system-packages，您可以将规则写入配置文件：

Instead of typing `--break-system-packages` every time you run the `pip install` command, you can write the rules to a configuration file:

```bash
mkdir -p ~/.config/pip
```

```bash
echo -e "[global]\nbreak-system-packages=true" > ~/.config/pip/pip.conf
```

这些命令将首先创建 ~/.config/pip 文件夹（如果不存在），然后创建 pip.conf 配置文件并将以下行写入其中：

These commands will first create the ~/.config/pip folder (if it does not exist), then create the pip.conf configuration file and add the following line to it:

```bash
[global]
break-system-packages=true
```

之后，您可以使用 pip install 命令安装包，就像在 Ubuntu 22.04 及更早版本中一样

Afterwards, you can install packages using the `pip install` command, just like in Ubuntu 22.04 and earlier.
