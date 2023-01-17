<script>
    /** @type {import('./$types').PageData} */
    export let data;
</script>

<div>
    <h1>Enimga</h1>
    <h2>Last encoded message:</h2>
    <p id="message">{data.messageFromEnigma}</p>
</div>

<style>
    main {
        text-align: center;
    }
</style>