# Front End

## Summary

**Hugo** (and its existing themes) is geared toward sites with a lot of markdown content. It's a simple publishing framework for markdown content. If I'm just posting articles, blogging, or making a simple personal/portfolio website, Hugo is a great choice.

But it's also highly customizable if you make your own theme. If you do, you can build literally any type of website with it. Although, for single page apps (SPAs) with highly dynamic content like dashboards or things like Google Maps, a **React** based framework and SSG is probably better.

**Gatsby:** If I need to create any custom UI (dashboard, custom layout, custom buttons, etc.), make a React app. There are tons of free UI component libraries for React. Try using **Gatsby** as the framework.

Top choice tech stacks to try:

* Normal website (blog, portfolio, business)
  * **Theme-based:** Hugo + Hugo theme (Congo, Hugo Fresh, Hextra, Doks)
    * Just create content, let theme do the design
  * **Custom:** Hugo +  **daisyUI,** Tailwind UI, **Mamba UI,** HyperUI
    * Create my own design using off-the-shelf UI components that can be dropped in (headers, footers, navbars, heroes, cards, etc.)
* Dynamic single-page app
  * Gatsby + Netlify + Ant Design, Radix, shadcn, NextUI
  * Copy/paste off-the-shelf UI components into site design

## JAM Stack

[https://jamstack.org/](https://jamstack.org/)

## SSGs (Static Site Generators)

### For React

* **Gatsby** | [https://www.gatsbyjs.com/](https://www.gatsbyjs.com/)
  * Gatsby + Netlify is recommended
* Next.js

### Non-React

* Hugo

## Other Popular Tools

* Vite

## UI Design Tools

### Levels

* **Level 1** (lowest)
  * Pure CSS
* **Level 2** - utility classes
  * **Tailwind CSS -** basically group CSS settings into class names to use in HTML; move CSS into to HTML
  * **Semantic UI** - higher level than Tailwind, UI component level
* **Level 3** - UI component libraries
  * On top of Tailwind
    * **daisyUI** - semantically named classes, things like "primary button", etc., use the class names in HTML
  * On top of pure CSS
    * **Bulma**
    * **Bootstrap** (try others before this, others seem nicer)
  * On top of React
    * **Ant Design**
    * **Radix**
    * **shadcn**
    * **NextUI**

## CSS Utility Classes

* **Tailwind CSS** | [https://tailwindcss.com/](https://tailwindcss.com/)

## UI Component Libraries

* **Creative Tim** | [https://www.creative-tim.com/](https://www.creative-tim.com/)
  * React version | [https://www.creative-tim.com/product/argon-design-system-react](https://www.creative-tim.com/product/argon-design-system-react)
  * Tons of stuff
  * Free dashboard templates, Tailwind UI components, etc.

### React Based

* :star: **Ant Design** | [https://ant.design/](https://ant.design/)
* :star: **Radix** | [https://www.radix-ui.com/](https://www.radix-ui.com/)
  * Focus on accessibility
* :star: **shadcn** | [https://ui.shadcn.com/](https://ui.shadcn.com/)
  * Tailwind based
  * Copy and paste code directly into your codebase, component by component
  * No package to install
* :star: **NextUI** | [https://nextui.org/](https://nextui.org/)
  * Beautiful components
  * Figma integration
* **Aceternity UI** | [https://ui.aceternity.com/](https://ui.aceternity.com/)
  * Super fancy 3D animation components
* **chakra** | [https://v2.chakra-ui.com/](https://v2.chakra-ui.com/)
  * **Choc UI** | [https://choc-ui.com/](https://choc-ui.com/)
    * UI components for **chakra**
* **Tremor** | [https://www.tremor.so/](https://www.tremor.so/)
  * For building charts and dashboards
* **Blueprint JS** | [https://blueprintjs.com/](https://blueprintjs.com/)
* **PrimeReact** | [https://primereact.org/](https://primereact.org/)
* **Evergreen** | [https://evergreen.segment.com/](https://evergreen.segment.com/)

### Non-React Based

These are good candidates for Hugo since they don't require React.

* :star: **Bulma** | [https://bulma.io/](https://bulma.io/)
  * Looks awesome
  * Pure CSS, single CSS file to add to your project
* **Semantic UI** | [https://semantic-ui.com/](https://semantic-ui.com/)
  * React version | [https://react.semantic-ui.com/](https://react.semantic-ui.com/)
* **MUI (Material UI)** | [https://mui.com/](https://mui.com/)
  * Google-style UI components for free
* **Bootstrap** | [https://getbootstrap.com/](https://getbootstrap.com/)
  * **Tabler** | [https://tabler.io/](https://tabler.io/)
    * UI kit template for dashboard apps
    * Bootstrap 5 based
    * Also offer illustrations, email templates, **free icons**
* **UIkit** | [https://getuikit.com/](https://getuikit.com/)
  * Just add css and js files to project
* **Tachyons** | [https://tachyons.io/](https://tachyons.io/)
  * Pure CSS

### Tailwind Based, Non-React

These are good candidates for Hugo since they don't require React.

* :star: **daisyUI** | [https://daisyui.com/](https://daisyui.com/)
  * Looks incredible, top choice to try
* :star: **Mamba UI** | [https://mambaui.com/](https://mambaui.com/)
* :star: **HyperUI** | [https://www.hyperui.dev/](https://www.hyperui.dev/)
* **Tailwind UI** | [https://tailwindui.com/](https://tailwindui.com/)
  * Makers of Tailwind CSS
  * Some free, some paid
  * Components and full UI layouts/templates
  * React and Vue compatible, but I don't think they're required
* **uiverse** | [https://uiverse.io/](https://uiverse.io/)
* **FloatUI** | [https://floatui.com/](https://floatui.com/)
* **tailwindcraft** | [https://tailwindcraft.com/](https://tailwindcraft.com/)
  * HTML + Tailwind
* **Wicked Blocks** | [https://wickedblocks.dev/](https://wickedblocks.dev/)
* **TailGrids** | [https://tailgrids.com/](https://tailgrids.com/)

## Static Site Deployment Platforms

* **Netlify** | [https://www.netlify.com/](https://www.netlify.com/)
  * Seems extremely popular, every SSG and framework points you to it, integrates with all of them
* **Cloudflare Pages** | [https://pages.cloudflare.com/](https://pages.cloudflare.com/)
  * I'm using it currently, it's great, super fast and easy to use, excellent
  * Includes many other cloud features like storage buckets, serverless functions (workers), etc.
  * All cloud services seem to have a generous free tier
* **Vercel** | [https://vercel.com/](https://vercel.com/)
  * Really interesting, seems to have a lot of features, would like to try
* Laravel Vapor?
  * Mentioned in a YouTube video, haven't looked into it much, seems relatively popular

## IDEs

* **MarsX** | [https://www.marsx.dev/](https://www.marsx.dev/)
