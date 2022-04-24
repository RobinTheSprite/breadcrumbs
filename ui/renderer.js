function submitParams(event, formElement) {
  event.preventDefault()

  let data = new FormData(formElement)
  window.api.generateRoute(data)
}

let form = document.getElementById("parameters-form")
form.addEventListener("submit", event => submitParams(event, form))
