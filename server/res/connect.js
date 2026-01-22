const ws = new WebSocket(`ws://${window.location.host}`)

var lastType = ""

ws.onmessage = msg => {
    if (msg.data instanceof Blob) {
        document.getElementById("view").src = URL.createObjectURL(msg.data);
    } else {
        console.log(msg.data)
    }
}

document.getElementById("send").onclick = () => ws.send("promote")
