function toggleTheme() {
    document.body.classList.toggle('dark');
  }
  
  function fetchStatus() {
    fetch('/status')
      .then(response => response.json())
      .then(data => {
        document.getElementById("mode").textContent = data.mode;
        document.getElementById("blinds").textContent = data.blinds_closed
          ? "Closed"
          : "Open " + data.manual_position + "%";
  
        for (let i = 0; i < 4; i++) {
          if (document.getElementById("sensor" + i)) {
            document.getElementById("sensor" + i).textContent = data.light_sensors[i];
          }
        }
      });
  }
  
  function setPosition() {
    const pos = document.getElementById("positionSlider").value;
    fetch('/position', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: 'value=' + pos
    }).then(fetchStatus);
  }
  
  function updateSettings() {
    const openVal = document.getElementById("openThreshold").value;
    const closeVal = document.getElementById("closeThreshold").value;
    fetch('/settings', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: `open=${openVal}&close=${closeVal}`
    });
  }
  