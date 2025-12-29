local this_file = debug.getinfo(1, "S").source:sub(2)
local project_root = vim.fn.fnamemodify(this_file, ":h")

-- print("🚀 Project config loaded. Root: " .. project_root)

local function resolve_script(script_rel_path)
	if script_rel_path:sub(1, 1) ~= "/" then
		script_rel_path = "/" .. script_rel_path
	end

	local full_path = project_root .. script_rel_path
	local abs_path = vim.fn.fnamemodify(full_path, ":p"):gsub("\\", "/") -- 统一斜杠

	if vim.fn.filereadable(abs_path) == 0 then
		error(
			string.format(
				"❌ File not found:\n" .. "  Requested: %s\n" .. "  Full path: %s\n" .. "  Exists?   %s",
				script_rel_path,
				abs_path,
				vim.fn.filereadable(abs_path) == 1 and "YES" or "NO"
			),
			2
		)
	end

	if vim.fn.executable(abs_path) == 0 then
		error("❌ No execute permission: " .. abs_path, 2)
	end

	return abs_path
end

local function run_script(script_rel_path)
	return function()
		local ok, script_path = pcall(resolve_script, script_rel_path)

		if not ok then
			vim.notify(script_path, vim.log.levels.ERROR, { title = "Path Error" }) -- script_path 包含错误信息
			return
		end

		local cmd
		if vim.fn.has("win32") == 1 then
			cmd = string.format('"%s"', script_path) -- Windows 需要引号
		else
			cmd = string.format("'%s'", script_path:gsub("'", "'\\''")) -- 安全转义单引号
		end

		local output = vim.fn.system(cmd)
		vim.cmd.redraw()

		if vim.v.shell_error ~= 0 then
			vim.notify("🔥 Command failed (" .. vim.v.shell_error .. "):\n" .. output, vim.log.levels.ERROR)
		else
			print("✅ Success:\n" .. output:gsub("\n$", ""))
		end
	end
end

local function run_script_with_digit(script_rel_path)
	return function()
		vim.fn.inputsave()
		local char = vim.fn.nr2char(vim.fn.getchar())
		vim.fn.inputrestore()

		if not char:match("^%d$") then
			vim.notify("🔢 Expected digit (0-9) after mapping", vim.log.levels.WARN)
			return
		end

		local ok, script_path = pcall(resolve_script, script_rel_path)

		if not ok then
			vim.notify(script_path, vim.log.levels.ERROR, { title = "Path Error" })
			return
		end

		local cmd
		if vim.fn.has("win32") == 1 then
			cmd = string.format('"%s" %s', script_path, char)
		else
			cmd = string.format("'%s' %s", script_path:gsub("'", "'\\''"), char)
		end

		local output = vim.fn.system(cmd)
		vim.cmd.redraw()

		if vim.v.shell_error ~= 0 then
			vim.notify("🔥 Upload failed (" .. char .. "):\n" .. output, vim.log.levels.ERROR)
		else
			print(string.format("✅ Slot %s:\n%s", char, output:gsub("\n$", "")))
		end
	end
end

local opts = { silent = true }

vim.keymap.set("n", "<leader>ua", run_script_with_digit("a.sh"), opts)
vim.keymap.set("n", "<leader>ub", run_script_with_digit("b.sh"), opts)
vim.keymap.set("n", "<leader>ur", run_script_with_digit("run.sh"), opts)
vim.keymap.set("n", "<leader>uu", run_script_with_digit("upload.sh"), opts)
vim.keymap.set("n", "<leader>uc", run_script("build.sh"), opts)
