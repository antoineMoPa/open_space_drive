
var assetMenuApp = new Vue({
  el: '#asset-picker',
  template: `
    <div>
      <button class="toggle-button" style="right:0;"
              v-on:click="shown = !shown">|||</button>
      <transition name="menu-pop">
        <div class="asset-picker ui-box" v-if="shown">
          <h1>Asset picker</h1>
          <div>
            <button v-for="asset in assetsList"
                    class="preview-button"
                    v-on:mousedown="addAsset(asset)">
              <img v-bind:src="'../assets/'+asset+'/preview/preview.png'"
                   v-bind:assetName="asset"
                   v-on:mouseenter="mouseEnter"
                   v-on:mouseleave="mouseLeave">
              Add '{{asset}}'
            </button>
          </div>
        </div>
      </transition>
    </div>
  `,
  data: {
    shown: false,
    assetsList: []
  },
  methods: {
    addAsset(assetName){
      window.addAsset(assetName);
      window.focus();
    },
    mouseEnter(e){
      e.target.src = "../assets/" + e.target.getAttribute("assetName") + "/preview/preview.gif";
    },
    mouseLeave(e){
      e.target.src = "../assets/" + e.target.getAttribute("assetName") + "/preview/preview.png";
    }
  },
  mounted(){
    this.assetsList = window.assets.assets;
  }
});
