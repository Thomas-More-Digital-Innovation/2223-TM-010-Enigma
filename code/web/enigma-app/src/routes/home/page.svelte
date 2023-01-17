<script lang="ts">
	const returnEnigmaMessage(){
		const messageFromEnigma = (await platform.env.MESSAGE.get('message')) || '0';
	    return messageFromEnigma;
	};
</script>

<main>
    <h1>Enimga</h1>
    <h2>Last encoded message:</h2>
    <p id="message"></p>
    <p>{returnEnigmaMessage()}</p>
</main>


<style>
    main{
        text-align: center;
    }
</style>