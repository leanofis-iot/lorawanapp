const {Button, TextInput, TextView, Composite, CollectionView, ui} = require('tabris')

console.log(screen.width)
console.log(screen.height)

let config = [
  ['dev_eui', 'XXXXXXXXXXXXXXXX'],
  ['app_eui', 'XXXXXXXXXXXXXXXX'],
  ['app_key', 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX'],
  ['pow_tm', 'XXXXXXXXXXXXXXXX'],
  ['bat_lo_v', 'XXXXXXXXXXXXXXXX'],
  ['interface', 'XXXXXXXXXXXXXXXX'],
  ['sensor', 'XXXXXXXXXXXXXXXX'],
  ['ntc_tmp', 'XXXXXXXXXXXXXXXX'],
  ['ntc_res', 'XXXXXXXXXXXXXXXX'],
  ['ntc_beta', 'XXXXXXXXXXXXXXXX'],
  ['ntc_ser_res', 'XXXXXXXXXXXXXXXX'],
  ['an_num_samp', 'XXXXXXXXXXXXXXXX'],
  ['an_dly_samp', 'XXXXXXXXXXXXXXXX'],
  ['an_alr_en', 'XXXXXXXXXXXXXXXX'],
  ['an_alr_hi_set', 'XXXXXXXXXXXXXXXX'],
  ['an_alr_hi_clr', 'XXXXXXXXXXXXXXXX'],
  ['an_alr_lo_set', 'XXXXXXXXXXXXXXXX'],
  ['an_alr_lo_clr', 'XXXXXXXXXXXXXXXX'],
  ['dig_alr_hi_en', 'XXXXXXXXXXXXXXXX'],
  ['dig_alr_lo_en', 'XXXXXXXXXXXXXXXX']
  ].map(([name, value]) => ({name, value}))

let open = false

new CollectionView({
  left: 0, top: 80, right: 0, height: 250, background: 'red',
  itemCount: config.length,
  cellHeight: 120,
  createCell: () => {
    let cell = new Composite()
    new TextView({
      centerX: 0, top: 'prev() 30', alignment: 'center', font: 'bold 24px'
    }).appendTo(cell)
    new TextInput({
      centerX: 0, top: 'prev() 30', alignment: 'center'
    }).appendTo(cell)
    return cell
  },
  updateCell: (cell, index) => {
    let param = config[index]
    cell.apply({
        TextView: {text: param.name},
        TextInput: {text: param.value}
    })
  }
}).on('select', ({index}) => console.log('selected', config[index].name))
  .appendTo(ui.contentView)

let connect = new Button({
    top: 20, left: 40, width: 80, text: 'Connect'   
}).appendTo(ui.contentView)
let receive = new Button({
    top: 20, left: 140, width: 80, text: 'Receive'   
}).appendTo(ui.contentView)
let send = new Button({
    top: 20, left: 240, width: 80, text: 'Send'   
}).appendTo(ui.contentView)


/*
let tx_data = new TextInput({
    top: 150, left:100, text: 'at+version', background: 'yellow'
}).appendTo(ui.contentView)

let  rx_data = new TextView({
    top: 200, left:100, text: 'hello', background: 'red'
}).appendTo(ui.contentView)

function errorCallback (message) {
    console.log(message)
}

serial.requestPermission(
    {driver: 'Ch34xSerialDriver'},
    function(successMessage) {
    	serial.open(
        	{baudRate: 115200},
            function(successMessage) {
                open = true                
                console.log(successMessage)
                console.log('ready')
                serial.registerReadCallback(
                    function success(data){  
                        let view = new Uint8Array(data);
                        console.log(view); 
                        console.log(String.fromCharCode.apply(null, view))                     
                        console.log('callback works')
                    },
                    function error(){
                        console.log("Failed to register read callback")
                    } 
                )

        	},
        	errorCallback
    	)
    },
    errorCallback
)
send.on('select', () => {
    console.log('send it')       
    //let str = 'at+join=otaa' + String.fromCharCode(13, 10)
    let str = 'at+send=0,1,01670110' + '\r\n'
    serial.write(str)     
})
*/
