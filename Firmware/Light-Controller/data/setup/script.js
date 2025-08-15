document.getElementById("wifiForm").addEventListener("submit", function(e) {
    e.preventDefault();
    
    let formData = new FormData(this);
    let params = new URLSearchParams();
    for (let pair of formData.entries()) {
        params.append(pair[0], pair[1]);
    }

    fetch("/save", {
        method: "POST",
        body: params.toString(),
        headers: { "Content-Type": "application/x-www-form-urlencoded" }
    })
    .then(response => response.text())
    .then(text => {
        document.getElementById("status").textContent = "Configuration saved! Rebooting...";
        setTimeout(() => {
            location.reload();
        }, 2000);
    })
    .catch(err => {
        document.getElementById("status").textContent = "Error saving configuration.";
    });
});
