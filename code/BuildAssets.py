from AssetsPreviewer import PreviewApp
import os
import json

with open("assets_manager/assets-list.js","w") as assetsListFile:
  assetsListFile.write("var assets = " + json.dumps({"assets": os.listdir("assets")}) + ";")

app = PreviewApp()
