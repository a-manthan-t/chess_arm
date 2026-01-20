const ws = new WebSocket(`ws://${window.location.host}`)

ws.onmessage = msg => console.log(msg.data)

document.getElementById("send").onclick = () => ws.send("promote")
