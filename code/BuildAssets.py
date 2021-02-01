from AssetsPreviewer import PreviewApp
import os
import json

with open("ui/assets-list.js","w") as assetsListFile:
  assetsListFile.write("var assets = " + json.dumps({"assets": os.listdir("assets")}) + ";")

app = PreviewApp()
