
var currentAssetApp = new Vue({
  el: '#current-asset-manager',
  template: `
    <div class="current-asset-manager ui-box">
      <h1>Object</h1>
      <div v-if="asset != null">
        <h1>{{asset.path}}</h1>
        <span>(uuid: {{asset.uuid}})</span><br/><br/>
        <button v-on:click="deleteAsset(asset.uuid)" class="danger-button">
          Delete object
        </button>
      </div>
      <div v-else-if="nearestAsset != null">
        <h1>(uuid: {{nearestAsset.path}})</h1>
        <span>{{nearestAsset.uuid}}</span><br/><br/>
        <button v-on:click="selectAsset(nearestAsset.uuid)">
          Select
        </button><br/>
        <button v-on:click="deleteAsset(nearestAsset.uuid)" class="danger-button">
          Delete
        </button>
      </div>
    </div>
`,
  data: {
    asset: null,
    nearestAsset: null
  },
  methods: {
    selectAsset(uuid){
      window.selectAsset(uuid);
    },
    deleteAsset(uuid){
      window.deleteAsset(uuid);
      this.asset = null;
      this.nearestAsset = null;
    }
  }
});


function onNearestAsset(asset) {
  currentAssetApp.nearestAsset = asset;
}

function onAssetSelected(asset) {
  currentAssetApp.asset = asset;
}
