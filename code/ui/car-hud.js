
new Vue({
  el: '#car-hud',
  template: `
    <div>
      <div class="car-control">
        <button v-on:click="alignCar">
          <img src="images/ui-align-car-1.svg" height="100px" width="100px">
        </button>
      </div>
    </div>
`,
  data: {

  },
  methods: {
    alignCar(){
      window.alignCar();
    }
  }
});
