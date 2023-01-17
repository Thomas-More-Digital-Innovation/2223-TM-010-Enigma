<script>
	import type { PageData } from './$types';

    export let data: PageData;

</script>

<main>
    <h1>Enimga</h1>
    <h2>Last encoded message:</h2>
    <p id="message">{data.messageFromEnigma}</p>

</main>


<style>
    main{
        text-align: center;
    }
</style>