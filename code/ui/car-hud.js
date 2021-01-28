
let carHudApp = new Vue({
  el: '#car-hud',
  template: `
    <div>
      <div class="car-control">
        <button v-on:click="alignCar">
          <img src="images/ui-align-car-1.svg" height="100px" width="100px">
        </button>
      </div>
      <p class="speed-indicator">{{speed}}<span class="unit">km/h</span></p>
    </div>
`,
  data: {
    speed: 0
  },
  methods: {
    alignCar(){
      window.alignCar();
    },
    onSpeed(_speed){
      this.speed = _speed;
    }
  }
});


window.onCarSpeed = carHudApp.onSpeed;
