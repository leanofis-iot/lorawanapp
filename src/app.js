const {Button, TextInput, TextView, ui} = require('tabris');

let open = false

let send = new Button({
    top: 100, left:100   
}).appendTo(ui.contentView)

let tx_data = new TextInput({
    top: 150, left:100
}).appendTo(ui.contentView)

let  rx_data = new TextView({
    top: 200, left:100
}).appendTo(ui.contentView)

function errorCallback (message) {
    new AlertDialog({
        title: 'Error',
        message: message,
        buttons: {ok: 'OK'}
    }).open() 
}

serial.requestPermission(
    function(successMessage) {
    	serial.open(
        	{baudRate: 9600},
            function(successMessage) {
                open = true                
                new AlertDialog({
                    title: 'Port Open',
                    message: successMessage,
                    buttons: {ok: 'OK'}
                }).open()
                serial.registerReadCallback(
                    function success(data){
                        let view = new Uint8Array(data)
                        rx_data.text = view
                    },
                    function error(){
                        new AlertDialog({
                            title: 'Error',
                            message: "Failed to register read callback",
                            buttons: {ok: 'OK'}
                        }).open()
                    } 
                )

        	},
        	errorCallback
    	)
    },
    errorCallback
)
send.on('select', () => {
    console.log('click')
    if (open) serial.write(tx_data.text + '\r\n');
})
