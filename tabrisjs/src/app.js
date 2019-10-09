const {Button, TextInput, TextView, Composite, ui} = require('tabris')

let config = {
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

let confIndex = 0

function errorCallback (message) {
    console.log(message)
}

let conBut = new Button({
  top: 5, left: 40, width: 80, text: 'Connect'   
}).appendTo(ui.contentView)
let confRecvBut = new Button({
  top: 145, left: 100, width: 80, text: 'Receive'   
}).appendTo(ui.contentView)
let confSendBut = new Button({
  top: 145, left: 200, width: 80, text: 'Send'   
}).appendTo(ui.contentView)
let confRwdBut = new Button({
  top: 60, left: 40, width: 80, text: 'Rwd'   
}).appendTo(ui.contentView)
let confFwdBut = new Button({
  top: 60, left: 240, width: 80, text: 'fwd'   
}).appendTo(ui.contentView)

conBut.on('select', () => {
  serial.requestPermission(
    {driver: 'Ch34xSerialDriver'},
    function(successMessage) {
    	serial.open(
        {baudRate: 115200}, // default parser \n
        function(successMessage) {
          console.log(successMessage)
          serial.registerReadCallback(
            function success(data){  
              let array = new Uint8Array(data)
              let str = String.fromCharCode.apply(null, array) 
              str = str.trim() 
              console.log(str)            
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

confRecvBut.on('select', () => {
  ui.contentView.find(Composite).dispose()
  config = {} 
  confIndex = 0 
  serial.write('send' + '\r\n')
})

confSendBut.on('select', () => {
  for (let key in config) {
    serial.write(key + '=' + config[key] + '\r\n')
    //delay
  }
  serial.write('save' + '\r\n')
})

confRwdBut.on('select', () => {
  confIndex--
  if (confIndex < 0) confIndex = 0
  ui.contentView.find(Composite).set('visible', false)
  ui.contentView.find('#' + confIndex).set('visible', true)
})

confFwdBut.on('select', () => {
  confIndex++
  if (confIndex > Object.keys(config).length - 1) confIndex = Object.keys(config).length - 1 
  ui.contentView.find(Composite).set('visible', false)
  ui.contentView.find('#' + confIndex).set('visible', true)
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
  let i = 0
  for (let key in config) {
    let cell = new Composite({
      id: i,
      top: 60, left: 0, right: 0, height: 140, visible: false, background: 'gray'
    }).appendTo(ui.contentView)
    new TextView({
      top: 0, left: 0, right: 0, height: 30, alignment: 'center',
      text: key, font: 'bold 24px'
    }).appendTo(cell)
    new TextInput({
      top: 40, left: 10, right: 10, height: 50, alignment: 'center',
      text: config[key]
    }).on('textChanged', (target, value) => config[key] = value)
    .appendTo(cell)
    i++
  }
  ui.contentView.find('#0').set('visible', true)  
}

createCells()