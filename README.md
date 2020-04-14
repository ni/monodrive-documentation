# monoDrive Docs

## Getting Started

- Clone project

```bash
$ git clone git@github.com:monoDriveIO/documentation.git
```

- Create python environment 

```bash
$ conda env create -f environment.yml
$ conda activate monodrive-documentation
```

- Docs are created using [mkdocs](https://www.mkdocs.org) with the theme [*ivory*](https://github.com/daizutabi/mkdocs-ivory)

- Run in development on [http://127.0.0.1:8000/](http://127.0.0.1:8000/)

```bash
(monodrive-documentation) $ mkdocs serve
```

<p>&nbsp;</p>


## Build

- Login/create user to [readthedocs](https://readthedocs.org/) *preferably through github for easy access to monodrive*

- Get access to be a maintainer through someone who is already a maintainer. (like me, [Katie](katie@monodrive.io))

- After having access, click on >> My Projects >> monoDrive

- Select **latest** and Build Version.

<p>&nbsp;</p>


## Navigation / Side Panel

Add new sections and documents to  `mkdocs.yml` with their title and corresponding file from the docs folder.

```
- monoDrive :
  - Home: 
    - monoDrive: index.md
```

<p>&nbsp;</p>

## Markdown & Styling

Consistency with the markdown effects the layout of the side panel as well as the over all consistency between sections. Here are some recommended guidelines:

### Headers

```markdown
# MarkDown Header Example Title

## Markdown Header Example Title (repeat)

Repeat the same title twice, once for the overall section, and one for the basic details of the section.

## Additional Header Example Information

Headers need to go in order, so for instance, resist going from a h2 ## to an h4 ####. This might break the styling of the side panel. 
Any sub-sections that get to h4 or smaller will not be included in the side panel, and will show the same as h3 ###. This probably indicates a need to break down the information into more manageable chunks for the user. 

### Sub-Header is Seen and Clickable in Side Panel

#### This Sub-Header is Not Seen in Side Panel
```

<p>&nbsp;</p>

### Images
```html
    <div class="img_container">
        <img class='lg_img' src="path/to/image.png"/>
    </div>
```
#### Sizing and Centering
To center an image, wrap in a div with the class img_container like in the example above. Four separate sizes were created using css:

 - thumbnail  { width: 50px }

        <img class="thumbnail" src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

    <img width=50px src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

 - sm_img { width: 200px }

        <img class="sm_img" src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

    <img width=200px src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

 - lg_img { width: 400px }

        <img class="lg_img" src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

    <img width=400px src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

 - wide_img { width: 90% }

        <img class="wide_img" src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

    <img width=90% src= https://github.com/monoDriveIO/client/raw/master/WikiPhotos/monoDriveLogo.png alt="monoDrive"/>

Add custom image sizes in [extra.css](https://github.com/monoDriveIO/documentation/raw/master/docs/css/extra.css)

<p>&nbsp;</p>


### Recommendations

- Add a line between sections : `<p>&nbsp;</p>`

- There should always be an empty line before bulleted lists 

- For instructions, add lines between list items

- Capitalize all header titles

<p>&nbsp;</p>


## Sources

Created using [mkdocs](https://www.mkdocs.org) with the theme [*ivory*](https://github.com/daizutabi/mkdocs-ivory)

Written in [markdown](https://www.markdownguide.org/)

Hosted through [readthedocs](https://readthedocs.org/)
