
var assetMenuApp = new Vue({
  el: '#asset-picker',
  template: `
    <div class="asset-picker ui-box">
      <h1>Asset picker</h1>
      <div>
        <button class="preview-button"
                v-on:click="addAsset('palm_tree')">
          <img src="../assets/palm_tree/preview/preview.png"
               v-on:mouseenter="mouseEnter"
               v-on:mouseleave="mouseLeave">
          Add palm tree
        </button>
      </div>
    </div>
  `,
  data: {
    imageIndex: 0
  },
  methods: {
    addAsset(assetName){
      window.addAsset(assetName);
    },
    mouseEnter(e){
      e.target.src = "../assets/palm_tree/preview/preview.gif";
    },
    mouseLeave(e){
      e.target.src = "../assets/palm_tree/preview/preview.png";
    }
  }
});
