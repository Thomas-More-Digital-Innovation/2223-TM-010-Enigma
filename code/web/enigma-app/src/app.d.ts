// See https://kit.svelte.dev/docs/types#app
// for information about these interfaces
// and what to do when importing types

declare namespace App {
// interface Error {}
// interface Locals {}
// interface PageData {}
// interface Platform {}
	interface Platform {
		env: {
			MESSAGE: KVNamespace;
			API_TOKEN: String;
		};
		context: {
			waitUntil(promise: Promise<any>): void;
		};
		caches: CacheStorage & { default: Cache }
	}
}

