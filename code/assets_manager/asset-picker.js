
var assetMenuApp = new Vue({
  el: '#asset-picker',
  template: `
    <div class="asset-picker ui-box">
      <h1>Asset picker</h1>
      <div>
        <button class="preview-button"
                v-on:click="addAsset('palm_tree')">
          <img src="../assets/palm_tree/preview.png">
          Add palm tree
        </button>
      </div>
    </div>
  `,
  data: {
  },
  methods: {
    addAsset(assetName){
      window.addAsset(assetName);
    },
  }
});
