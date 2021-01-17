
var assetMenuApp = new Vue({
  el: '#asset-picker',
  template: `
    <div class="asset-picker ui-box">
      <h1>Asset picker</h1>
      <div>
        <button v-on:click="addAsset('palm_tree')">Add palm tree</button>
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
