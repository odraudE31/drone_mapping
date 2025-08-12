# drone_mapping

#### 🧑‍💻 Code usage

- To use the python code, just run the mapping code on the SBC in the drone.
- Lua code need further research and testing

#### 🌎 Qground Usage

Use the `Survey` mode, and set parameters so you have a correct height and speed for the perfect map.

#### 🗺️ WebODM 

**Installation**
```bash
git clone https://github.com/OpenDroneMap/WebODM --config core.autocrlf=input --depth 1
cd WebODM
./webodm.sh start
sudo usermod -aG docker $USER
exit
# (restart shell by logging out and then back-in)
./webodm.sh start
```

**Usage**

Open a Web Browser to [http://localhost:8000](http://localhost:8000) and generate map over the images. On WebODM, after starting the new project choose Fast Orthophoto. Click Edit and set the specifications:

*Parameters used:*
- Fatest obtained:
```
auto-boundary:true,fast-orthophoto:true,feature-quality:ultra,orthophoto-resolution:2,pc-quality:low,skip-3dmodel:true
```

#### 🗾 QGIS Installation and Usage 

**Installation**

Follow the steps in `https://qgis.org/download/` to install on your machine

**Usage**

To bring together different `.tif` ortophotos:
```
Layer > Add Layer > Add Raster Layer 
```
Then choose the `.tif` ortophoto you want to add.