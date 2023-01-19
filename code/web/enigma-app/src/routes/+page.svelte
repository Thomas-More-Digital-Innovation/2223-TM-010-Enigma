<script>
    /** @type {import('./$types').PageData} */
    export let data;
</script>
<main>
<div>
    <!-- <h1>Enimga</h1> -->
    <img src="favicon.png" alt="">
    <h2>Last encoded message:</h2>
    <p id="message">{data.messageFromEnigma}</p>
    
</div>
</main>

<style>
    @import url('https://fonts.googleapis.com/css2?family=Roboto:wght@400;700&display=swap');
    main {
        text-align: center;
        font-family: 'Roboto', sans-serif;
        /* background-color: #808080;
        color: white; */
    }
    h2{
        font-weight: 700;
        font-size: 2.5rem;
    }
    p{
        font-weight: 400;
        font-size: 2.5rem;
    }
</style>