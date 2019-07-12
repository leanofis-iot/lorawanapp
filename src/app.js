const {Button, TextInput, TextView, Composite, WidgetCollection, ui} = require('tabris')

console.log(screen.width)
console.log(screen.height)

let config = {
  'dev_eui': 'ZZZZZZ',
  'app_eui': 'hjadsfjXXX',
  'app_key': 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX',
  'pow_tm': 'XXXXXXXXXXXXXXXX',
  'bat_lo_v': 'XXXXXXXXXXXXXXXX',
  'interface': 'XXXXXXXXXXXXXXXX',
  'sensor': 'XXXXXXXXXXXXXXXX',
  'ntc_tmp': 'XXXXXXXXXXXXXXXX',
  'ntc_res': 'XXXXXXXXXXXXXXXX',
  'ntc_beta': 'XXXXXXXXXXXXXXXX',
  'ntc_ser_res': 'XXXXXXXXXXXXXXXX',
  'an_num_samp': 'XXXXXXXXXXXXXXXX',
  'an_dly_samp': 'XXXXXXXXXXXXXXXX',
  'an_alr_en': 'XXXXXXXXXXXXXXXX',
  'an_alr_hi_set': 'XXXXXXXXXXXXXXXX',
  'an_alr_hi_clr': 'XXXXXXXXXXXXXXXX',
  'an_alr_lo_set': 'XXXXXXXXXXXXXXXX',
  'an_alr_lo_clr': 'XXXXXXXXXXXXXXXX',
  'dig_alr_hi_en': 'yryryryryXXXXXXX',
  'dig_alr_lo_en': 'SSSSS'
}

let open = false
let index = 0

function errorCallback (message) {
    console.log(message)
}

let conBut = new Button({
  top: 20, left: 40, width: 80, text: 'Connect'   
}).appendTo(ui.contentView)
let recvBut = new Button({
  top: 20, left: 140, width: 80, text: 'Receive'   
}).appendTo(ui.contentView)
let sendBut = new Button({
  top: 20, left: 240, width: 80, text: 'Send'   
}).appendTo(ui.contentView)
let rwdBut = new Button({
  top: 230, left: 90, width: 80, text: 'Rwd'   
}).appendTo(ui.contentView)
let fwdBut = new Button({
  top: 230, left: 190, width: 80, text: 'fwd'   
}).appendTo(ui.contentView)

conBut.on('select', () => {
  serial.requestPermission(
    {driver: 'Ch34xSerialDriver'},
    function(successMessage) {
    	serial.open(
        {baudRate: 115200},
        function(successMessage) {
          open = true                
          console.log(successMessage)
          serial.registerReadCallback(
            function success(data){  
              let array = new Uint8Array(data)
              let str = String.fromCharCode.apply(null, array) 
              str.replace('\r\n', '')             
              onRecv(str)                     
            },
            function error(){
              console.log("Failed to register read callback")
            } 
          )
        },
        errorCallback()
      )
    },
    errorCallback()
  )
})

recvBut.on('select', () => {
  config = {}
  serial.write('send' + '\r\n')
})

sendBut.on('select', () => {
  for (let key in config) {
    serial.write(key + '=' + config[key] + '\r\n')
    //delay
  }
  serial.write('save' + '\r\n')
})

rwdBut.on('select', () => {
  index--
  if (index < 0) index = 0
  ui.contentView.find(Composite).set('visible', false)
  ui.contentView.find('#' + index).set('visible', true)
})

fwdBut.on('select', () => {
  index++
  if (index > config.length) index = config.length
  ui.contentView.find(Composite).set('visible', false)
  ui.contentView.find('#' + index).set('visible', true)
})

function onRecv(recvStr) {
  if (recvStr = 'save') {
    createCells()
  } else {
    let str = recvStr.split('=')
    for (let key in config) {
      if (key = str[0]) {
        config[key] = str[1]
      }
    }
  }
}

function createCells() {
  ui.contentView.find(Composite).dispose()
  index = 0
  let i = 0
  for (let key in config) {
    let cell = new Composite({
      id: i,
      top: 100, left: 0, right: 0, height: 120, visible: false, background: 'red'    
    }).appendTo(ui.contentView)
    new TextView({
      top: 0, left: 0, right: 0, height: 50, alignment: 'center',
      text: key, font: 'bold 24px'
    }).appendTo(cell)
    new TextInput({
      top: 50, left: 10, right: 10, height: 50, alignment: 'center',
      text: config[key]
    }).on('textChanged', (target, value) => config[key] = value)
    .appendTo(cell)
    i++
  }
  ui.contentView.find('#0').set('visible', true)  
}